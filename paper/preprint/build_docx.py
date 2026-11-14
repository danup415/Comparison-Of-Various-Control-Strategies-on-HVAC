"""Build manuscript_draft.docx from main.tex with pandoc (pip install pypandoc_binary).
LaTeX remains the master; compile main.tex with pdflatex for the PDF."""
import os, re, subprocess, pypandoc
here = os.path.dirname(os.path.abspath(__file__)); os.chdir(here)
def expand(path):
    p = path if path.endswith('.tex') else path + '.tex'
    return re.sub(r'\\input\{([^}]+)\}', lambda m: expand(m.group(1)), open(p).read())
src = expand('main.tex').replace(r'\usepackage[hidelinks]{hyperref}', '')
# pandoc's OMML writer fails on labelled display equations in a numbered document: drop eq labels
src = re.sub(r'\\label\{eq:[^}]*\}', '', src)
src = re.sub(r'Eqs?\.~\\eqref\{eq:[^}]*\}(--\\eqref\{eq:[^}]*\})?', 'the equations above', src)
r = subprocess.run([pypandoc.get_pandoc_path(), '-f', 'latex', '-t', 'docx', '-o', 'manuscript_draft.docx',
                    '--resource-path=../../results/figures:.', '--number-sections'], input=src.encode(), capture_output=True)
print(r.stderr.decode() or 'ok')
