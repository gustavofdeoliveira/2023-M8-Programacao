#!/bin/bash

chmod +x setup.sh

sudo apt-get install xterm

# Comando para criar o modelo
ollama create dexter -f Modelfile

# Comando para executar o modelo
ollama run dexter
