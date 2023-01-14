# Wrist-Based Sign Language Detection and Communication

## Installation

**Install Python 3**

Run the following:
```bash
python3 -m pip install -r requirements.txt
```

**Install chocolatey**

Open powershell as administrator and run the following:
```bash
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
```

**Install linting tools**

Open command prompt as administrator and run the following:
```bash
choco install llvm uncrustify

pre-commit install
```

**Install ARM Embedded Toolchain**

Install version 9-2020-q2-update from: https://developer.arm.com/downloads/-/gnu-rm
