#you can use this function to find a folder named with ${suiteSparseNameFolder}
#NOTE suitesparse_dir is the output variable with path to the input folder
set(nameFolder "SuiteSparse")
function(find_dir)
  find_dir_full("/home" FALSE)
endfunction()

function(find_dir_full dir is_found)
  file(GLOB allfiles RELATIVE ${dir} ${dir}/*)
  foreach(file ${allfiles})
    if(NOT is_found)
      if(IS_DIRECTORY ${dir}/${file})
        string(COMPARE EQUAL ${file} ${nameFolder} is_equal)
        if(is_equal)
          set(is_found TRUE)
          set(dir_path ${dir}/${file} CACHE INTERNAL "" FORCE)
          message("found! ${dir}/${file}")
        else()
          find_dir_full(${dir}/${file} ${is_found})
        endif()
      endif()
    endif()
  endforeach()
endfunction()

#########################################################################
if(NOT suitesparse_dir)
  message("\nPath to SuiteSparse is not set. Please type:")
  message("$ make SUITESPARSE_DIR=<path to SuiteSparse>")
  message("  ex make SUITESPARSE_DIR=home/user/SuiteSparse\n")
  message("try to find SuiteSparse by own...")
  find_dir()
  set(suitesparse_dir ${dir_path})
endif()
if(suitesparse_dir)
  #file(GLOB children ${SUITESPARSE_DIR}/*)
  list(APPEND children ${suitesparse_dir}/CHOLMOD
                       ${suitesparse_dir}/CSparse
                       ${suitesparse_dir}/SuiteSparse_config
                       ${suitesparse_dir}/COLAMD
                       ${suitesparse_dir}/AMD
                       ${suitesparse_dir}/UFMPACK)
  foreach(child ${children})
    if(IS_DIRECTORY ${child})
      list(APPEND CHOLMODINSUITESPARSE_INCLUDES ${child} ${child}/Include)
      file(GLOB_RECURSE lib_in_dir ${child}/*.a)
      list(APPEND CHOLMODINSUITESPARSE_LIBRARIES ${lib_in_dir})
    endif()
  endforeach()
  # link liblapack.a and libblas.a from /user/lib
  # NOTE: if you don't have this libraries in /user/lib, you should download it
  # $ sudo apt-get install liblapack3
  # $ sudo apt-get install libblas3
  list(APPEND CHOLMODINSUITESPARSE_LIBRARIES lapack blas)
endif()
#########################################################################
