from openai import OpenAI
from dotenv import load_dotenv
from openai import AssistantEventHandler
from typing_extensions import override
load_dotenv()
client = OpenAI()
file = client.files.create(
  file=open("benchmark.jpeg", "rb"),
  purpose="vision"
)

thread = client.beta.threads.create( messages=[
    {
      "role": "user",
      "content": [
        {
          "type": "text",
          "text": "Pick the paper and place it in the container"
        },
     
        {
          "type": "image_file",
          "image_file": {"file_id": file.id}
        },
      ],
    }
  ])


class EventHandler(AssistantEventHandler):    
  @override
  def on_text_created(self, text) -> None:
    print(f"\nassistant > ", end="", flush=True)
      
  @override
  def on_text_delta(self, delta, snapshot):
    print(delta.value, end="", flush=True)

with client.beta.threads.runs.stream(
  thread_id=thread.id,
  assistant_id="asst_jD1C0Xx5vXXD17Y6R33gAdiS",
  event_handler=EventHandler(),
) as stream:
  stream.until_done()
