# VORA
### Authors: [Dexter Friis-Hecht](https://github.com/dfriishecht), Dokyun Kim, [Dominic Salmieri](https://github.com/joloujo), [Maurice Ampane](https://github.com/Moampane)

Visit our [project website](https://dokyun-kim4.github.io/vora)

## Troubleshooting
Listed below are common error messages we encountered while running this on Ubuntu and their troubleshooting guides.  
1. `ERROR: Could not build wheels for pyaudio, which is required to install pyproject.toml-based projects`  
   Solution:  
    ```console
    sudo apt install portaudio19-dev
    ```
3. `OSError: libespeak.so.1: cannot open shared object file: No such file or directory`  
   Solution:  
   ```console
   sudo apt install espeak
   ```
