#!/bin/bash
cd "$(dirname "$0")"

TEMP="temp"

ls ./templates | while read FILE
do
    TRIMMED="${FILE%.*}" # remove file extension
    DEST="compiled/$TRIMMED.xml"

    # replace token with actual partial
    sed -e "s|!!TEMPLATE_FILE!!|$TRIMMED|g" frames/MujocoGeneral.xml > "$TEMP.xml"

    hbs -P "{partials/*,templates/*}" "$TEMP.xml" 
    mv "$TEMP.html" "$DEST"
done

rm "$TEMP.xml"
