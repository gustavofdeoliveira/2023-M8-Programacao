import random
import gradio as gr
from langchain.llms import Ollama
from langchain.prompts import ChatPromptTemplate
from langchain.schema.runnable import RunnableMap, RunnablePassthrough

model = Ollama(model="dolphin2.2-mistral")
prompt = ChatPromptTemplate.from_template(
"""
Now you are an occupational {safety agent} specialized in safety standards
in industrial environments,in addition to knowing everything about epis and
other subjects related to occupational safety and industrial environments
that require a higher level of safety
"""
)

chain = {"safety agent": RunnablePassthrough()} | prompt | model


def response(message, history):
    response = ""
    for response_message in chain.stream(message):
        response += response_message
        yield response

def main():
    gradio = gr.ChatInterface(response).queue()

    gradio.launch()


if __name__ == "__main__":
    main()