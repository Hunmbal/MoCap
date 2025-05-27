# Button.py
import tkinter as tk


def toggle_recording():
    """Flip our recording flag and update the button text."""
    global recording, btn
    recording = not recording
    btn.config(text = "STOP" if recording else "START")

def gui_thread():
    """Simple Tk window with one big Start/Stop button."""
    global btn
    root = tk.Tk()
    root.title("Data Recorder")
    btn = tk.Button(root,
                    text="START",
                    font=("Arial", 24),
                    width=10,
                    height=2,
                    bg="green",
                    fg="white",
                    command=toggle_recording)
    btn.pack(padx=20, pady=20)
    root.mainloop()