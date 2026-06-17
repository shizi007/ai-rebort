@echo off
set IDF_PATH=D:\esp554\Espressif\frameworks\esp-idf-v5.5.4
set IDF_PYTHON_ENV_PATH=D:\esp554\Espressif\python_env\idf5.5_py3.11_env
set PATH=D:\esp554\Espressif\tools\cmake\3.30.2\bin;D:\esp554\Espressif\tools\ninja\1.12.1;D:\esp554\Espressif\tools\idf-git\2.44.0\cmd;%PATH%
cd /d D:\xiaozhi-esp32
D:\esp554\Espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe %IDF_PATH%\tools\idf.py build
