#!/bin/bash
cd "$(dirname "$0")"

ls ./templates | while read FILE
do
    . mo # This loads the "mo" function

    export name="" # TODO: get model name from file name
    frame=$(sed -e "s|!!TEMPLATE_FILE!!|$FILE|g" frames/MujocoGeneral.xml) # replace marker with actual template file

    echo "$frame" | mo > "compiled/$FILE"
done
