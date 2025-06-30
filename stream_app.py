import os

def reda_alarma():
    os.system("mpg123 /home/pi/alarma.mp3")

# Test:
if __name__ == "__main__":
    print("Redau alarma...")
    reda_alarma()
