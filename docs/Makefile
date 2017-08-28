# From: http://tex.stackexchange.com/questions/40738/how-to-properly-make-a-latex-project
# You want latexmk to *always* run, because make does not have all the info.
.PHONY: user_manual.pdf user_manual.ps.gz

# First rule should always be the default "all" rule, so both "make all" and
# "make" will invoke it.
all: user_manual.pdf

# MAIN LATEXMK RULE

# -pdf tells latexmk to generate PDF directly (instead of DVI).
# -pdflatex="" tells latexmk to call a specific backend with specific options.
# -use-make tells latexmk to call make for generating missing files.

# -interactive=nonstopmode keeps the pdflatex backend from stopping at a
# missing file reference and interactively asking you for an alternative.

user_manual.pdf: user_manual.tex
	latexmk -pdf -pdflatex="pdflatex -interactive=nonstopmode" -bibtex -use-make user_manual.tex

user_manual.ps.gz: user_manual.pdf
	pdf2ps user_manual.pdf
	gzip -f user_manual.ps

clean:
	latexmk -CA
