from PIL import Image
import pytesseract

pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files (x86)\\Tesseract-OCR\\tesseract.exe'

img = Image.open('images/ocr-test3.png')
text = pytesseract.image_to_string(img,config='-psm 10') 	 # config='-psm 10' -> treat as a single charachter [https://stackoverflow.com/questions/31643216/pytesseract-dont-work-with-one-digit-image]
print("text:", text)