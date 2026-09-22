include_guard(GLOBAL)
include(CMakeParseArguments)

# 解析别名链, 得到真正的实现 target(SOURCE 模式需要判断 TYPE/IMPORTED)。
function(bw_kinematics_resolve_implementation_target input_target output_variable)
  if(NOT TARGET "${input_target}")
    message(FATAL_ERROR "Implementation Provider target does not exist: ${input_target}")
  endif()

  set(resolved_target "${input_target}")
  while(TRUE)
    get_target_property(aliased_target "${resolved_target}" ALIASED_TARGET)
    if(NOT aliased_target OR aliased_target MATCHES "-NOTFOUND$")
      break()
    endif()
    set(resolved_target "${aliased_target}")
  endwhile()

  set(${output_variable} "${resolved_target}" PARENT_SCOPE)
endfunction()

# 建立 Provider 接口 target: 下游只依赖该 INTERFACE 目标, 不直接依赖实现库文件。
function(bw_kinematics_create_provider_interface
  provider_target export_name implementation_target public_include_dir export_set output_variable)
  if(TARGET ${provider_target})
    message(FATAL_ERROR
      "Implementation Provider target '${provider_target}' already exists")
  endif()

  add_library(${provider_target} INTERFACE)
  set_target_properties(${provider_target} PROPERTIES EXPORT_NAME ${export_name})
  target_include_directories(${provider_target} INTERFACE
    "$<BUILD_INTERFACE:${public_include_dir}>"
    "$<INSTALL_INTERFACE:include>")
  target_link_libraries(${provider_target} INTERFACE
    "$<BUILD_INTERFACE:${implementation_target}>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/lib${export_name}.so>")

  if(NOT TARGET ${export_name}::${export_name})
    add_library(${export_name}::${export_name} ALIAS ${provider_target})
  endif()
  if(NOT TARGET bw_kinematics::${export_name})
    add_library(bw_kinematics::${export_name} ALIAS ${provider_target})
  endif()

  install(TARGETS ${provider_target} EXPORT ${export_set})
  set(${output_variable} "${provider_target}" PARENT_SCOPE)
endfunction()

# 配置一个算法实现 Provider。
#
# NAME 决定 target/别名/变量前缀(如 algorithm 产生 BW_KINEMATICS_ALGORITHM_TARGET),
# MODE 为 PREBUILT 或 SOURCE; SOURCE 需要 SOURCE_DIR 提供独立 CMakeLists.txt。
function(bw_kinematics_configure_implementation_provider)
  set(options)
  set(one_value_args NAME MODE PUBLIC_INCLUDE_DIR PREBUILT_LIBRARY SOURCE_DIR EXPORT_SET)
  cmake_parse_arguments(PROVIDER "${options}" "${one_value_args}" "" ${ARGN})

  foreach(required_argument IN ITEMS NAME MODE PUBLIC_INCLUDE_DIR EXPORT_SET)
    if(NOT PROVIDER_${required_argument})
      message(FATAL_ERROR
        "bw_kinematics implementation provider requires ${required_argument}")
    endif()
  endforeach()

  string(TOLOWER "${PROVIDER_NAME}" provider_name)
  string(TOUPPER "${PROVIDER_NAME}" provider_name_upper)
  string(TOUPPER "${PROVIDER_MODE}" provider_mode)
  set(provider_target "bw_kinematics_${provider_name}_provider")
  set(export_name "bw_kinematics_${provider_name}")

  if(provider_mode STREQUAL "PREBUILT")
    if(NOT PROVIDER_PREBUILT_LIBRARY)
      message(FATAL_ERROR "PREBUILT mode requires PREBUILT_LIBRARY")
    endif()
    if(NOT EXISTS "${PROVIDER_PREBUILT_LIBRARY}")
      if(PROVIDER_SOURCE_DIR)
        set(source_hint
          " or use -DBW_KINEMATICS_${provider_name_upper}_MODE=SOURCE")
      endif()
      message(FATAL_ERROR
        "bw_kinematics PREBUILT ${provider_name} implementation library was not "
        "found: ${PROVIDER_PREBUILT_LIBRARY}\n"
        "Replace the file${source_hint}.")
    endif()

    set(provider_impl_target "bw_kinematics_${provider_name}_prebuilt")
    add_library(${provider_impl_target} SHARED IMPORTED GLOBAL)
    set_target_properties(${provider_impl_target} PROPERTIES
      IMPORTED_LOCATION "${PROVIDER_PREBUILT_LIBRARY}"
      IMPORTED_SONAME "lib${export_name}.so"
      INTERFACE_INCLUDE_DIRECTORIES "${PROVIDER_PUBLIC_INCLUDE_DIR}")

    get_filename_component(provider_library_dir "${PROVIDER_PREBUILT_LIBRARY}" DIRECTORY)
    bw_kinematics_create_provider_interface(
      "${provider_target}"
      "${export_name}"
      "${provider_impl_target}"
      "${PROVIDER_PUBLIC_INCLUDE_DIR}"
      "${PROVIDER_EXPORT_SET}"
      provider_interface_target)

    set(BW_KINEMATICS_${provider_name_upper}_TARGET
      "${provider_interface_target}" PARENT_SCOPE)
    set(BW_KINEMATICS_${provider_name_upper}_BUILD_TARGET "" PARENT_SCOPE)
    set(BW_KINEMATICS_${provider_name_upper}_BUILD_RPATH
      "${provider_library_dir}" PARENT_SCOPE)

    install(FILES "${PROVIDER_PREBUILT_LIBRARY}" DESTINATION lib)
    return()
  endif()

  if(provider_mode STREQUAL "SOURCE")
    if(NOT PROVIDER_SOURCE_DIR)
      message(FATAL_ERROR "SOURCE mode requires SOURCE_DIR")
    endif()
    if(NOT EXISTS "${PROVIDER_SOURCE_DIR}/CMakeLists.txt")
      message(FATAL_ERROR
        "bw_kinematics SOURCE mode requires: "
        "${PROVIDER_SOURCE_DIR}/CMakeLists.txt")
    endif()

    add_subdirectory(
      "${PROVIDER_SOURCE_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/bw_kinematics_${provider_name}_source")

    if(TARGET bw_kinematics_${provider_name})
      set(source_target bw_kinematics_${provider_name})
    elseif(TARGET bw_kinematics_${provider_name}::bw_kinematics_${provider_name})
      if(CMAKE_VERSION VERSION_LESS 3.18)
        message(FATAL_ERROR
          "SOURCE mode with a namespaced alias requires CMake 3.18 or newer; "
          "define the unnamespaced 'bw_kinematics_${provider_name}' target instead.")
      endif()
      set(source_target bw_kinematics_${provider_name}::bw_kinematics_${provider_name})
    else()
      message(FATAL_ERROR
        "${PROVIDER_SOURCE_DIR}/CMakeLists.txt must define the shared target "
        "'bw_kinematics_${provider_name}' or "
        "'bw_kinematics_${provider_name}::bw_kinematics_${provider_name}'.")
    endif()

    bw_kinematics_resolve_implementation_target(
      "${source_target}" provider_impl_target)

    get_target_property(provider_target_type "${provider_impl_target}" TYPE)
    if(NOT provider_target_type STREQUAL "SHARED_LIBRARY")
      message(FATAL_ERROR
        "The bw_kinematics SOURCE ${provider_name} target must be a SHARED "
        "library, got '${provider_target_type}'.")
    endif()

    get_target_property(provider_target_imported "${provider_impl_target}" IMPORTED)
    if(provider_target_imported AND NOT provider_target_imported MATCHES "-NOTFOUND$")
      message(FATAL_ERROR
        "The bw_kinematics SOURCE ${provider_name} target must be a build target, "
        "not an imported target.")
    endif()

    target_include_directories("${provider_impl_target}" PUBLIC
      "$<BUILD_INTERFACE:${PROVIDER_PUBLIC_INCLUDE_DIR}>"
      "$<INSTALL_INTERFACE:include>")
    set_target_properties("${provider_impl_target}" PROPERTIES
      OUTPUT_NAME "bw_kinematics_${provider_name}")

    bw_kinematics_create_provider_interface(
      "${provider_target}"
      "${export_name}"
      "${provider_impl_target}"
      "${PROVIDER_PUBLIC_INCLUDE_DIR}"
      "${PROVIDER_EXPORT_SET}"
      provider_interface_target)

    set(BW_KINEMATICS_${provider_name_upper}_TARGET
      "${provider_interface_target}" PARENT_SCOPE)
    set(BW_KINEMATICS_${provider_name_upper}_BUILD_TARGET
      "${provider_impl_target}" PARENT_SCOPE)
    set(BW_KINEMATICS_${provider_name_upper}_BUILD_RPATH
      "$<TARGET_FILE_DIR:${provider_impl_target}>" PARENT_SCOPE)

    install(TARGETS "${provider_impl_target}"
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin)
    return()
  endif()

  message(FATAL_ERROR
    "Unsupported ${provider_name} implementation mode '${PROVIDER_MODE}'. "
    "Use PREBUILT or SOURCE.")
endfunction()
