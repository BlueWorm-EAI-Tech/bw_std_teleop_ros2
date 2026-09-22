#include "bw_std_control/async_serial_transport.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <asio.hpp>

#include "bw_std_control/command_queue.hpp"

namespace bw_std_control
{
namespace
{

using namespace std::chrono_literals;

class SerialSession
{
public:
  SerialSession(
    const std::string & device, const std::uint32_t baud_rate,
    CommandMailbox & tx_mailbox,
    const std::uint64_t session_generation,
    AsyncSerialTransport::DataCallback data_callback,
    AsyncSerialTransport::ErrorCallback error_callback)
  : work_guard_(asio::make_work_guard(io_context_)),
    serial_port_(io_context_),
    tx_timer_(io_context_),
    tx_mailbox_(tx_mailbox),
    session_generation_(session_generation),
    data_callback_(std::move(data_callback)),
    error_callback_(std::move(error_callback))
  {
    serial_port_.open(device);
    serial_port_.set_option(asio::serial_port_base::baud_rate(baud_rate));
    serial_port_.set_option(asio::serial_port_base::flow_control(
        asio::serial_port_base::flow_control::none));
    serial_port_.set_option(asio::serial_port_base::parity(
        asio::serial_port_base::parity::none));
    serial_port_.set_option(asio::serial_port_base::stop_bits(
        asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(asio::serial_port_base::character_size(8));
    start_receive();
    schedule_tx_poll();
    thread_ = std::thread([this]() {io_context_.run();});
  }

  ~SerialSession()
  {
    close();
  }

  SerialSession(const SerialSession &) = delete;
  SerialSession & operator=(const SerialSession &) = delete;

  bool is_open() const noexcept
  {
    return open_.load(std::memory_order_acquire);
  }

  void close() noexcept
  {
    if (!open_.exchange(false, std::memory_order_acq_rel)) {
      if (thread_.joinable()) {
        io_context_.stop();
        thread_.join();
      }
      return;
    }
    try {
      if (thread_.joinable() && !io_context_.stopped()) {
        auto closed = std::make_shared<std::promise<void>>();
        auto future = closed->get_future();
        asio::post(
          io_context_,
          [this, closed]() {
            asio::error_code ignored;
            tx_timer_.cancel(ignored);
            serial_port_.cancel(ignored);
            CommandFrame power_off{};
            bool has_power_off = false;
            CommandFrame queued{};
            bool queued_is_power_off = false;
            std::uint64_t queued_sequence = 0U;
            while (tx_mailbox_.try_pop_next(
              queued, queued_is_power_off, queued_sequence, session_generation_))
            {
              if (queued_is_power_off) {
                power_off = queued;
                has_power_off = true;
              }
            }
            if (!has_power_off && tx_active_ && active_tx_[kPrefixSize] == 0U) {
              power_off = active_tx_;
              has_power_off = true;
            }
            // 生命周期关闭允许阻塞，尽最大努力发送最终软件掉电帧。
            if (has_power_off && serial_port_.is_open()) {
              asio::write(serial_port_, asio::buffer(power_off), ignored);
            }
            serial_port_.close(ignored);
            work_guard_.reset();
            closed->set_value();
          });
        future.wait();
      }
    } catch (...) {
      // 析构和停用路径不得抛异常，后续 stop/join 仍回收线程。
    }
    io_context_.stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  void start_receive()
  {
    serial_port_.async_read_some(
      asio::buffer(rx_buffer_),
      [this](const asio::error_code & error, const std::size_t transferred) {
        if (error) {
          if (error != asio::error::operation_aborted && is_open()) {
            error_callback_("serial receive failed: " + error.message());
          }
          return;
        }
        if (transferred > 0U) {
          data_callback_(rx_buffer_.data(), transferred);
        }
        if (is_open()) {
          start_receive();
        }
      });
  }

  void schedule_tx_poll()
  {
    tx_timer_.expires_after(1ms);
    tx_timer_.async_wait(
      [this](const asio::error_code & error) {
        if (error || !is_open()) {
          return;
        }
        start_pending_write();
        schedule_tx_poll();
      });
  }

  void start_pending_write()
  {
    if (tx_active_) {
      return;
    }
    if (!tx_mailbox_.try_pop_next(
        active_tx_, active_tx_is_power_off_, active_tx_sequence_, session_generation_))
    {
      return;
    }
    tx_active_ = true;
    // 帧长由协议类型决定: 整机 0x01 为 256 字节, 下盘 0x21 为 44 字节。
    const std::size_t tx_size = active_tx_[2] == kChassisCommandType ?
      kChassisCommandFrameSize : kCommandFrameSize;
    asio::async_write(
      serial_port_, asio::buffer(active_tx_.data(), tx_size),
      [this](const asio::error_code & error, const std::size_t) {
        const bool completed_power_off = active_tx_is_power_off_ && !error;
        const bool completed_normal = !active_tx_is_power_off_ && !error;
        const std::uint64_t completed_sequence = active_tx_sequence_;
        tx_active_ = false;
        active_tx_is_power_off_ = false;
        active_tx_sequence_ = 0U;
        if (completed_power_off) {
          tx_mailbox_.complete_power_off_from_consumer(
            completed_sequence, session_generation_);
        } else if (completed_normal) {
          tx_mailbox_.complete_normal_from_consumer(completed_sequence);
        }
        if (error && error != asio::error::operation_aborted && is_open()) {
          error_callback_("serial transmit failed: " + error.message());
        }
      });
  }

  asio::io_context io_context_;
  asio::executor_work_guard<asio::io_context::executor_type> work_guard_;
  asio::serial_port serial_port_;
  asio::steady_timer tx_timer_;
  CommandMailbox & tx_mailbox_;
  const std::uint64_t session_generation_;
  std::thread thread_;
  std::atomic_bool open_{true};
  std::array<std::uint8_t, 2048> rx_buffer_{};
  CommandFrame active_tx_{};
  bool tx_active_{false};
  bool active_tx_is_power_off_{false};
  std::uint64_t active_tx_sequence_{0U};
  AsyncSerialTransport::DataCallback data_callback_;
  AsyncSerialTransport::ErrorCallback error_callback_;
};

}  // namespace

class AsyncSerialTransport::Impl
{
public:
  void open(
    const std::string & device, const std::uint32_t baud_rate,
    DataCallback data_callback, ErrorCallback error_callback)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (active_session_generation_.load(std::memory_order_acquire) != 0U) {
      throw std::logic_error("serial transport is already open");
    }
    if (session_ != nullptr) {
      throw std::logic_error("serial transport is closing");
    }
    if (next_session_generation_ == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("serial transport session generation exhausted");
    }
    const std::uint64_t session_generation = next_session_generation_++;
    tx_mailbox_.clear_from_consumer();
    session_ = std::make_unique<SerialSession>(
      device, baud_rate, tx_mailbox_, session_generation,
      std::move(data_callback), std::move(error_callback));
    active_session_generation_.store(session_generation, std::memory_order_release);
  }

  void close() noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    active_session_generation_.store(0U, std::memory_order_release);
    session_.reset();
    tx_mailbox_.clear_from_consumer();
  }

  bool is_open() const noexcept
  {
    return active_session_generation_.load(std::memory_order_acquire) != 0U;
  }

  std::uint64_t async_write(const CommandFrame & frame) noexcept
  {
    const std::uint64_t session_generation =
      active_session_generation_.load(std::memory_order_acquire);
    if (session_generation == 0U) {
      return 0U;
    }
    const std::uint64_t sequence =
      tx_mailbox_.try_push_normal(frame, session_generation);
    return active_session_generation_.load(std::memory_order_acquire) == session_generation ?
           sequence : 0U;
  }

  bool write_completed(const std::uint64_t sequence) const noexcept
  {
    return tx_mailbox_.sequence_completed(sequence);
  }

  std::uint64_t async_write_power_off(const CommandFrame & frame) noexcept
  {
    const std::uint64_t session_generation =
      active_session_generation_.load(std::memory_order_acquire);
    if (session_generation == 0U) {
      return 0U;
    }
    const std::uint64_t sequence =
      tx_mailbox_.try_push_power_off(frame, session_generation);
    return active_session_generation_.load(std::memory_order_acquire) == session_generation ?
           sequence : 0U;
  }

  bool wait_for_power_off(
    const std::uint64_t sequence, const std::chrono::milliseconds timeout) noexcept
  {
    return tx_mailbox_.wait_for_power_off(sequence, timeout);
  }

private:
  mutable std::mutex mutex_;
  std::unique_ptr<SerialSession> session_;
  CommandMailbox tx_mailbox_{};
  std::uint64_t next_session_generation_{1U};
  std::atomic_uint64_t active_session_generation_{0U};
};

AsyncSerialTransport::AsyncSerialTransport()
: impl_(std::make_unique<Impl>())
{
}

AsyncSerialTransport::~AsyncSerialTransport() = default;

void AsyncSerialTransport::open(
  const std::string & device, const std::uint32_t baud_rate,
  DataCallback data_callback, ErrorCallback error_callback)
{
  impl_->open(device, baud_rate, std::move(data_callback), std::move(error_callback));
}

void AsyncSerialTransport::close() noexcept
{
  impl_->close();
}

bool AsyncSerialTransport::is_open() const noexcept
{
  return impl_->is_open();
}

std::uint64_t AsyncSerialTransport::async_write(const CommandFrame & frame) noexcept
{
  return impl_->async_write(frame);
}

bool AsyncSerialTransport::write_completed(const std::uint64_t sequence) const noexcept
{
  return impl_->write_completed(sequence);
}

std::uint64_t AsyncSerialTransport::async_write_power_off(
  const CommandFrame & frame) noexcept
{
  return impl_->async_write_power_off(frame);
}

bool AsyncSerialTransport::wait_for_power_off(
  const std::uint64_t sequence, const std::chrono::milliseconds timeout) noexcept
{
  return impl_->wait_for_power_off(sequence, timeout);
}

}  // namespace bw_std_control
