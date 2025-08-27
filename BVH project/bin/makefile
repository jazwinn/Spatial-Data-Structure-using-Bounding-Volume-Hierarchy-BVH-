TMP_FOLDER=.latex
INPUTS = $(wildcard ./.*.dot)
PNGS = $(patsubst %.dot,%.png,$(INPUTS))
SVGS = $(patsubst %.dot,%.svg,$(INPUTS))

.PHONY: all clean

all: $(PNGS) $(SVGS)

%.png: %.dot
	dot -O -Tpng $^ -O

%.svg: %.dot
	dot -O -Tsvg $^ -O

clean: 
	rm -f .*.png
	rm -f .*.dot
	rm -f .*.txt
