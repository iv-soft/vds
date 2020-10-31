set root_folder=%~d0%~p0
call setenv.bat

del %root_folder%\CMakeSettings.json
rmdir %root_folder%\build /s /q
mkdir %root_folder%\build
cd %root_folder%\build


rem cmake.exe  -DZLIB_INCLUDE_DIR=%root_folder%externals\zlib_out\include -DZLIB_LIBRARY=%root_folder%externals\zlib_out\lib\zlibstaticd.lib -DOPENSSL_ROOT_DIR=%root_folder%externals\build_openssl\ -DGTEST_LIBRARY=%root_folder%externals\build_gtest\lib\Debug\gtestd.lib -DGTEST_INCLUDE_DIR=%root_folder%externals\googletest\googletest\include -DGTEST_MAIN_LIBRARY=%root_folder%externals\build_gtest\lib\Debug\gtest_maind.lib ../ -G %project_style% -A %project_arch%
cmake.exe  -DZLIB_INCLUDE_DIR=%root_folder%externals\zlib_out\include -DZLIB_LIBRARY=%root_folder%externals\zlib_out\lib\zlibstaticd.lib -DOPENSSL_ROOT_DIR=%root_folder%externals\build_openssl\ -DGTEST_LIBRARY=%root_folder%externals\build_gtest\lib\Debug\gtestd.lib -DGTEST_INCLUDE_DIR=%root_folder%externals\googletest\googletest\include -DGTEST_MAIN_LIBRARY=%root_folder%externals\build_gtest\lib\Debug\gtest_maind.lib ../ -G %project_style% -A %project_arch%
cmake --build .

rem xcopy /s /y %root_folder%\vs.user\build %root_folder%\build 
rem start vds.sln 