import os
from dotenv import load_dotenv
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

from turbulence.turbulence import turbulence
from rtm.rtm import rtm

def repl():
  print("=== REPL ===")
  print("Commands: \n'rtm', 'turbulence', 'exit'")
  while True:
    try:
      user_input = input(">>> ").strip().lower()

      if user_input in ("exit", "quit"):
        print("Goodbye!")
        break

      if user_input == "rtm":
        rtm()
      elif user_input == "turbulence":
        turbulence()
      else:
        print(f"Unknown command '{user_input}'. Use 'rtm' or 'turbulence'.")

    except Exception as e:
      print(f"Error: {e}")

if __name__ == "__main__":
  repl()