from openai import OpenAI
import streamlit as st
import cv2shit as ass
# from dotenv import load_dotenv
# load_dotenv()
import ArduinoSerial as ar

st.title("PAPI-Physical API")
ass.cvshit("new3.jpeg")

client = OpenAI(api_key=st.secrets["OPENAI_API_KEY"])

file = client.files.create(
  file=open("new4.jpeg", "rb"),
  purpose="vision"
)

if "messages" not in st.session_state:
    st.session_state.messages = []
    thread = client.beta.threads.create()
    st.session_state.thread_id = thread.id
else:
    thread = client.beta.threads.retrieve(st.session_state.thread_id)


# for message in st.session_state.messages:
#     with st.chat_message(message["role"]):
#         st.markdown(message["content"])

if prompt := st.chat_input("What is up?"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    message = client.beta.threads.messages.create(
  thread_id=thread.id,
  role="user",
  content=prompt
)
    with st.chat_message("user"):
        st.markdown(prompt)

    with st.chat_message("assistant"):
        run = client.beta.threads.runs.create_and_poll(thread_id=thread.id,
  assistant_id="asst_jD1C0Xx5vXXD17Y6R33gAdiS",)
        while True:
            if run.status == 'completed': 
                messages = client.beta.threads.messages.list(thread_id=thread.id)
                break
        print(messages)
        response = messages.data[0].content[0].text.value
        # st.markdown(response)
        st.image("new4.jpeg",caption="Output after prediction")
        st.code(response, language="gcode")       
        print(response)
        res=list(response.split("\n"))
        print(res)
        for d in res:
            ar.sdgcode(d)
