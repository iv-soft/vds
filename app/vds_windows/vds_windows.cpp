// vds_windows.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "vds_windows.h"
#include "TrayIcon.h"

HANDLE execute_process(const char* process_name) {
  DWORD dwType;
  char ui_folder[FILENAME_MAX];
  DWORD cbData = sizeof(ui_folder);
  auto error = RegGetValue(HKEY_CURRENT_USER, "Software\\IVySoft\\VDS", "ui", RRF_RT_REG_SZ, &dwType, ui_folder, &cbData);
  if (ERROR_SUCCESS != error || dwType != REG_SZ) {
    return NULL;
  }

  char process_path[FILENAME_MAX];
  if (NULL == PathCombine(process_path, ui_folder, process_name)) {
    return NULL;
  }

  SHELLEXECUTEINFO se;
  memset(&se, 0, sizeof(se));
  se.cbSize = sizeof(SHELLEXECUTEINFO);
  se.fMask = SEE_MASK_NOCLOSEPROCESS;
  se.lpFile = process_path;
  se.lpParameters = "";
  se.lpDirectory = ui_folder;
  se.nShow = SW_SHOWNORMAL;
  if (ShellExecuteEx(&se)) {
    return se.hProcess;
  }

  return NULL;
}


// Forward declarations of functions included in this code module:
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY _tWinMain(
  _In_ HINSTANCE hInstance,
  _In_opt_ HINSTANCE hPrevInstance,
  _In_ LPTSTR    lpCmdLine,
  _In_ int       nCmdShow) {
  UNREFERENCED_PARAMETER(hPrevInstance);
  UNREFERENCED_PARAMETER(lpCmdLine);
  UNREFERENCED_PARAMETER(nCmdShow);

  if (0 == _tcscmp(_T("--stop"), lpCmdLine)) {
      const auto hWnd = FindWindowEx(NULL, NULL, TrayIcon::WndClassName, TrayIcon::WndWindowName);
      if (NULL != hWnd) {
          PostMessage(hWnd, WM_COMMAND, MAKEWPARAM(ID_POPUP_EXIT, 0), 0);
      }

      return 0;
  }

  TrayIcon::hAutoUpdateProcess = execute_process("IVySoft.VDS.Client.AutoUpdate.exe");

  TrayIcon trayMenu;
  if(!trayMenu.create(hInstance)) {
    return 2;
  }

  MSG msg;

  while (GetMessage(&msg, nullptr, 0, 0)) {
    TranslateMessage(&msg);
    DispatchMessage(&msg);
  }

  trayMenu.destroy();

  return (int)msg.wParam;
}

