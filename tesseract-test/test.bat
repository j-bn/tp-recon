:: --oem N
:: Specify OCR Engine mode. The options for N are:
:: 0 = Original Tesseract only.
:: 1 = Neural nets LSTM only.
:: 2 = Tesseract + LSTM.
:: 3 = Default, based on what is available.

:: --psm N
:: Set Tesseract to only run a subset of layout analysis and assume a certain form of image. The options for N are:
:: 0 = Orientation and script detection (OSD) only.
:: 1 = Automatic page segmentation with OSD.
:: 2 = Automatic page segmentation, but no OSD, or OCR.
:: 3 = Fully automatic page segmentation, but no OSD. (Default)
:: 4 = Assume a single column of text of variable sizes.
:: 5 = Assume a single uniform block of vertically aligned text.
:: 6 = Assume a single uniform block of text.
:: 7 = Treat the image as a single text line.
:: 8 = Treat the image as a single word.
:: 9 = Treat the image as a single word in a circle.
:: 10 = Treat the image as a single character.

tesseract test-0.bmp stdout --psm 0 -l eng
tesseract test-0.bmp stdout --psm 1 -l eng
tesseract test-0.bmp stdout --psm 5 -l eng
tesseract test-0.bmp stdout --psm 6 -l eng
tesseract test-0.bmp stdout --psm 10 -l eng
