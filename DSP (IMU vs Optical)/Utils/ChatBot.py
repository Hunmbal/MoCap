import os
import pyttsx3
import speech_recognition as sr
import pyaudio
import wave

# Recording settings
chunk = 1024
sample_format = pyaudio.paInt16
channels = 1
fs = 44100
seconds = 5
filename = "input.wav"

# Initialize TTS engine
engine = pyttsx3.init()

# Function to say a message
def say(text):
    os.system("cls")
    print(text)
    engine.say(text)
    engine.runAndWait()

# Function to ask a question and get the answer
def ask_and_get(text, answer="yes"):
    say(text)
    print("(yes/no)?")
    
    # Record the answer after asking the question
    listen()
    
    recognizer = sr.Recognizer()
    try:
        with sr.AudioFile(filename) as source:
            audio = recognizer.record(source)
        response = recognizer.recognize_google(audio).strip().lower()
        if "yes" in response or "start" in response or "ok" in response or "yep" in response:
            return True
    except sr.UnknownValueError:
        print("Sorry, I did not understand that.")
    except sr.RequestError:
        print("Sorry, there was a problem with the speech recognition service.")
    except FileNotFoundError:
        print("Audio file not found. Please make sure the file exists and is accessible.")
    
    return False

# Function to record audio
def listen():
    print('\n-> Listening....')
    
    p = pyaudio.PyAudio()
    stream = p.open(format=sample_format, channels=channels, rate=fs, input=True, frames_per_buffer=chunk)
    
    frames = []
    for _ in range(0, int(fs / chunk * seconds)):
        data = stream.read(chunk)
        frames.append(data)
    
    stream.stop_stream()
    stream.close()
    p.terminate()
    
    print('Finished listening')
    
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(p.get_sample_size(sample_format))
        wf.setframerate(fs)
        wf.writeframes(b''.join(frames))

# Example prompts implemented as functions
def main():
    say("Calibration file not found. Entering calibration.")
    say("Calibration file exists. Loading calibration data.")
    
    if ask_and_get("Would you still like to calibrate? (yes/no): "):
        say("Starting calibration process.")
    else:
        say("Using old calibration data.")

    if ask_and_get("Are you ready? (yes/no): "):
        say("Great, let's continue!")
    else:
        say("Waiting for you to be ready.")

if __name__ == "__main__":
    main()
