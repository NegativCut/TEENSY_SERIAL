---
name: PDF parser — pymupdf
description: How to extract text from PDF datasheets on this machine
type: reference
---

`pymupdf` (import as `fitz`) is installed at Python 3.11 site-packages.

```python
import fitz
doc = fitz.open("path/to/file.pdf")
print(doc.page_count)
text = doc[page_index].get_text()
```

Confirmed working on `DATASHEET/ST7735S.pdf` (201 pages).

Use via Bash: `python -c "import fitz; ..."` or a temporary script.
