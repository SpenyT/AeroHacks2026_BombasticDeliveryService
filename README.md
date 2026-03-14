# AeroHacks2026_BombasticDeliveryService


# Developer guide
 
## 1. Create the Virtual Environment
 
Run this command in your project root:
 
```bash
python -m venv venv
```
 
> This creates the python environment.

---
 
## 2. Activate the Virtual Environment
 
### Linux / macOS
 
```bash
source venv/bin/activate
```
 
### Windows (Command Prompt)
 
```cmd
venv\Scripts\activate.bat
```
 
### Windows (PowerShell)
 
```powershell
venv\Scripts\Activate.ps1
```
 
> **PowerShell tip:** If you get a permission error, run this first:
> ```powershell
> Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
> ```
 
Once activated, your terminal prompt will show `(venv)` at the beginning:
 
```
(venv) user@machine:~/project$
```
 
---
 
## 3. Install from `requirements.txt`
 
With the virtual environment active, run:
 
```bash
pip install -r requirements.txt
```
 
---
 
## 4. Deactivate the Virtual Environment
 
When you're done, simply run:
 
```bash
deactivate
```

# Run Project
From the project root:
```
python src/main.py
```
Or to run as a module:
```
python -m src.main
```