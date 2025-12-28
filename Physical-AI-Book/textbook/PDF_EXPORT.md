# PDF Export Guide

This document explains how to generate a PDF version of the Physical AI & Humanoid Robotics textbook.

## Overview

Docusaurus does not include built-in PDF export. However, several methods are available to generate professional PDF documentation from the built static site.

## Recommended Methods

### Method 1: Print to PDF (Quick & Simple)

**Best for**: Quick review copies, single-page PDFs

1. Start the development server:
   ```bash
   npm start
   ```

2. Navigate to http://localhost:3000/docs/preface/

3. Use browser print functionality:
   - Chrome/Edge: Menu → Print → Save as PDF
   - Set margins to "None" for full-width content
   - Enable "Background graphics" for diagrams
   - For single-module PDFs, print each module separately

**Limitations**: Each page is a separate PDF; requires manual merging

---

### Method 2: Docusaurus-to-PDF Plugin (Automated)

**Best for**: Full textbook PDF with table of contents

**Installation**:
```bash
npm install --save-dev docusaurus-prince-pdf
```

**Configuration** (add to `docusaurus.config.js`):
```javascript
plugins: [
  [
    'docusaurus-prince-pdf',
    {
      princeOptions: { pdf: { printBackground: true } },
      output: 'physical-ai-robotics-textbook.pdf',
    },
  ],
],
```

**Generate PDF**:
```bash
npm run build
npx docusaurus-prince-pdf -u http://localhost:3000/docs/preface/
```

**Requirements**: Prince XML (commercial license required for production use)

---

### Method 3: Puppeteer-based Automation (Free)

**Best for**: Automated CI/CD pipeline, free solution

**Create script** (`scripts/generate-pdf.js`):
```javascript
const puppeteer = require('puppeteer');
const PDFMerger = require('pdf-merger-js');

const pages = [
  'http://localhost:3000/docs/preface/',
  'http://localhost:3000/docs/foundations-ros2',
  'http://localhost:3000/docs/digital-twin',
  'http://localhost:3000/docs/ai-robot-brain',
  'http://localhost:3000/docs/vision-language-action',
  'http://localhost:3000/docs/appendices/glossary',
];

(async () => {
  const browser = await puppeteer.launch();
  const merger = new PDFMerger();

  for (let i = 0; i < pages.length; i++) {
    const page = await browser.newPage();
    await page.goto(pages[i], { waitUntil: 'networkidle0' });
    const pdf = `temp-${i}.pdf`;
    await page.pdf({
      path: pdf,
      format: 'A4',
      printBackground: true,
      margin: { top: '1cm', right: '1cm', bottom: '1cm', left: '1cm' },
    });
    await merger.add(pdf);
  }

  await merger.save('physical-ai-robotics-textbook.pdf');
  await browser.close();
})();
```

**Install dependencies**:
```bash
npm install --save-dev puppeteer pdf-merger-js
```

**Run**:
```bash
# Terminal 1: Start dev server
npm start

# Terminal 2: Generate PDF
node scripts/generate-pdf.js
```

---

### Method 4: Pandoc Conversion (Markdown → PDF)

**Best for**: Academic formatting, LaTeX-style output

**Requirements**: Pandoc + LaTeX (texlive-full)

**Install**:
```bash
sudo apt install pandoc texlive-full
```

**Convert**:
```bash
# Combine all markdown files
cat docs/00-preface/index.md \
    docs/01-foundations-ros2/*.md \
    docs/02-digital-twin/*.md \
    docs/03-ai-robot-brain/*.md \
    docs/04-vision-language-action/*.md \
    > full-textbook.md

# Generate PDF with custom template
pandoc full-textbook.md \
  --pdf-engine=xelatex \
  --toc --toc-depth=3 \
  --number-sections \
  --highlight-style=tango \
  -V geometry:margin=1in \
  -V fontsize=11pt \
  -V documentclass=report \
  -V papersize=letter \
  -o physical-ai-robotics-textbook.pdf
```

**Note**: Requires manual cleanup of Docusaurus-specific syntax (frontmatter, MDX components)

---

## Recommended Production Workflow

### For Instructors/Publishers:

1. **Build static site**:
   ```bash
   npm run build
   npm run serve
   ```

2. **Use Method 2 (docusaurus-prince-pdf)** for professional output
   - Includes TOC with page numbers
   - Preserves syntax highlighting
   - Handles multi-column layouts correctly

3. **Alternative**: Use professional PDF generation service
   - Upload built site to: https://www.pdflayer.com/
   - Or: https://pdfcrowd.com/
   - Configure for single-page scrollable PDF

### For Students (Free):

1. Use **Method 1 (Print to PDF)** for individual modules
2. Or use **Method 3 (Puppeteer)** for full textbook
3. Merge PDFs using: https://www.ilovepdf.com/merge_pdf

---

## PDF Formatting Considerations

### Preserve Formatting:
- SVG diagrams render correctly (vector graphics scale)
- Code blocks maintain syntax highlighting
- Tables auto-fit to page width

### Known Limitations:
- Interactive elements (expanding sections) become static
- Dark mode toggle not included in PDF
- Search functionality unavailable (use PDF reader's search)

### Recommended PDF Settings:
- Paper size: US Letter or A4
- Margins: 0.75 inches (1.9 cm)
- Print background graphics: **Enabled**
- Headers/footers: Module name + page number

---

## License Compliance

When distributing PDF exports:

1. Include license notice on title page:
   ```
   This work is licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
   https://creativecommons.org/licenses/by-nc-sa/4.0/
   ```

2. Attribution requirements:
   - Original author: [Your name/institution]
   - Title: Physical AI & Humanoid Robotics
   - Source: [URL to repository]

3. NonCommercial clause:
   - Free for educational use
   - Contact author for commercial licensing

---

## Testing PDF Export

### Validation Checklist:

- [ ] Table of contents renders correctly
- [ ] All diagrams appear (5 SVG files)
- [ ] Code blocks preserve syntax highlighting
- [ ] Internal links work (if PDF supports hyperlinks)
- [ ] Module navigation is clear
- [ ] License notice included on title page
- [ ] Page numbers present
- [ ] No truncated content

### Quality Checks:

1. **Visual inspection**: Open PDF and skim all modules
2. **Accessibility**: PDF is tagged for screen readers (use Adobe Acrobat Pro)
3. **File size**: Should be 5-15 MB (text + diagrams)
4. **Metadata**: Author, title, subject fields populated

---

## Automation (CI/CD)

### GitHub Actions Workflow:

```yaml
name: Generate PDF

on:
  release:
    types: [published]

jobs:
  pdf-export:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm install
      - run: npm run build
      - run: npm run serve &
      - run: sleep 10
      - run: npm install -g docusaurus-prince-pdf
      - run: docusaurus-prince-pdf -u http://localhost:3000/docs/preface/ --output textbook.pdf
      - uses: actions/upload-artifact@v3
        with:
          name: textbook-pdf
          path: textbook.pdf
```

---

## Support

For PDF export issues:
- Docusaurus docs: https://docusaurus.io/docs
- Puppeteer docs: https://pptr.dev/
- Pandoc manual: https://pandoc.org/MANUAL.html

**Last updated**: December 2025
