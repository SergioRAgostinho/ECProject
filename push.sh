#!/bin/bash
# usage: ./push.sh "Mensagem de commit"
# $1 - mensagem de commit

git add -A
git commit -m $1
git push -u origin master

