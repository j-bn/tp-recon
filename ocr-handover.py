# the actual code that runs Tesseract from Python 3.6.x is something like this....
# ---------------

import pytesseract

# tesseract installed from https://github.com/UB-Mannheim/tesseract/wiki (3.05...)
pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files (x86)\\Tesseract-OCR\\tesseract.exe'

# pass to Tesseract OCR engine
# config='-psm 10' -> treat as a single charachter
# https://stackoverflow.com/questions/31643216/pytesseract-dont-work-with-one-digit-image
# https://stackoverflow.com/questions/44619077/pytesseract-ocr-multiple-config-options
# however, with testing 6 seems to work better
ocrConfig = '-psm 6' # could also use PSM 10

ocrText = pytesseract.image_to_string(imgD, config=ocrConfig)