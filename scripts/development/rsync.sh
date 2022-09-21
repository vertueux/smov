#!/bin/bash

cd ~/smov || exit

#For example
#export PASSWORD=""
#export REMOTE_FOLDER="franferri@192.168.1.104:/Users/franferri/projects/basic-runtime/"

if [ -z ${PASSWORD+x} ]; then
  echo "PASSWORD is unset"
  echo "Use 'export PASSWORD=\"XXXXXX\"' to set it"
  exit
else echo "PASSWORD is set"; fi

if [ -z ${REMOTE_FOLDER+x} ]; then
  echo "REMOTE_FOLDER is unset."
  echo "Use 'export REMOTE_FOLDER=\"username@ip:/folderto/basic-runtime/\"' to set it"
  echo "For example 'export REMOTE_FOLDER=\"franferri@192.168.1.104:/Users/franferri/projects/basic-runtime/\""
  exit
else echo "REMOTE_FOLDER is set to '$REMOTE_FOLDER'"; fi

sshpass -p "$PASSWORD" rsync -avz -e "ssh -o StrictHostKeyChecking=no" --delete --exclude '.git' --exclude-from ~/smov/.gitignore "$REMOTE_FOLDER" ~/smov/
