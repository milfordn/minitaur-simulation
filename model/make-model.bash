#!/bin/bash
cd "$(dirname "$0")"

TEMP="temp"

ls ./templates | while read FILE
do
    TRIMMED="${FILE%.*}" # remove file extension
    DEST="compiled/$TRIMMED.xml"

    # replace token with actual partial reference
    sed -e "s|!!TEMPLATE_FILE!!|$TRIMMED|g" frames/MujocoGeneral.xml &> "$TEMP.xml"

    hbs -P "{partials/*,templates/*}" "$TEMP.xml" > /dev/null
    echo "compiled $FILE" 
    mv "$TEMP.html" "$DEST"
done

rm "$TEMP.xml"
