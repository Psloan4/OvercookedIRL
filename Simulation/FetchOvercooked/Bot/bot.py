import time
import pyautogui

# Wait 2 seconds
time.sleep(2)

# Press and hold 'd'
pyautogui.keyDown('d')

# Hold for 5 seconds
time.sleep(5)

# Release 'd'
pyautogui.keyUp('d')