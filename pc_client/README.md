# SCARA PC Client

This directory contains the PC-side TCP client. It uses only the Python standard
library, so no additional package installation is required.

From the project root, start the terminal with:

```powershell
python pc_client/scara_terminal.py
```

Press Enter to use the displayed default IP, or enter the current IP printed by
the ESP32 `wifiStatus` command. After connecting, enter SCARA commands directly:

```text
SCARA> setScaraAngles 20 -30
QUEUED
DONE
```

Enter `exit`, `quit`, or press Ctrl+C to close the client.

`scara_client.py` contains the reusable TCP layer intended for both this terminal
and a future Qt GUI.
