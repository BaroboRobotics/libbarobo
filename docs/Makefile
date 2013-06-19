mobot.pdf: mobot.tex
	pdflatex mobot.tex
	makeindex mobot.idx
	pdflatex mobot.tex
	cp mobot.pdf barobo.pdf
clean:
	rm -rf mobot.pdf mobot.ind
