# Deployment Guide - Physical AI & Humanoid Robotics Textbook

## Quick Start

### Development Server
```bash
cd /home/sohaib/hackathon/ai-book/textbook/textbook
npm start
```
Opens browser at http://localhost:3000

### Production Build
```bash
npm run build
```
Generates static files in `build/` directory

### Test Production Build Locally
```bash
npm run serve
```
Serves the built site at http://localhost:3000

## Deployment Options

### 1. GitHub Pages
```bash
# Set correct base URL in docusaurus.config.js
# Then deploy:
npm run deploy
```

### 2. Netlify/Vercel
- Connect GitHub repository
- Build command: `npm run build`
- Publish directory: `build`

### 3. Static Hosting (AWS S3, nginx)
```bash
npm run build
# Upload contents of build/ to your hosting
```

## File Structure

```
textbook/
├── docs/                    # All markdown content
│   ├── 00-preface/         # Introduction, prerequisites, ethics
│   ├── 01-foundations-ros2/ # Module 1: ROS 2 Middleware
│   ├── 02-digital-twin/    # Module 2: Simulation
│   ├── 03-ai-robot-brain/  # Module 3: GPU & Isaac
│   ├── 04-vision-language-action/ # Module 4: VLA Systems
│   ├── 99-appendices/      # Glossary, references
│   └── diagrams/           # SVG diagrams
├── build/                  # Generated static site (after build)
├── docusaurus.config.js    # Main configuration
├── sidebars.js            # Navigation structure
└── package.json           # Dependencies

```

## Instructor Features

### Answer Files Location
```
docs/01-foundations-ros2/.answers/exercises-answers.md
docs/02-digital-twin/.answers/exercises-answers.md
docs/03-ai-robot-brain/.answers/exercises-answers.md
docs/04-vision-language-action/.answers/exercises-answers.md
```

**IMPORTANT**: Answer directories are EXCLUDED from student-facing sidebar navigation.

### Making Answers Public (Optional)
Edit `sidebars.js` to include `.answers` directories if you want to publish instructor solutions.

## Customization

### Site Metadata
Edit `docusaurus.config.js`:
```javascript
title: 'Your Title',
tagline: 'Your Tagline',
url: 'https://yourdomain.com',
baseUrl: '/',
```

### Theme Colors
Edit `src/css/custom.css` for color scheme adjustments.

### Adding New Modules
1. Create directory: `docs/05-new-module/`
2. Add chapters with frontmatter
3. Update `sidebars.js`
4. Add to module index in `docs/`

## Validation

### Build Test
```bash
npm run build
# Should complete with "Generated static files in 'build'."
```

### Link Checking
```bash
npm run build 2>&1 | grep "Broken link"
# Review and fix any broken internal links
```

### Markdown Linting (Optional)
```bash
npm install -g markdownlint-cli
markdownlint docs/**/*.md
```

## Performance

- Build time: ~15-30 seconds (all modules)
- Page load: <2 seconds (static site)
- Search: Built-in Algolia DocSearch (configure in config)

## Troubleshooting

### MDX Compilation Errors
- Escape `<` symbols: use `&lt;` instead of `<` outside code blocks
- Escape `{` symbols: use `\{` in text content
- Check line numbers in error messages

### Build Fails
```bash
rm -rf .docusaurus build node_modules
npm install
npm run build
```

### Missing Diagrams
Verify SVG files exist in `docs/diagrams/` and are referenced correctly in markdown.

## License

CC BY-NC-SA 4.0 International - See LICENSE file

## Support

For issues, refer to:
- Docusaurus docs: https://docusaurus.io/docs
- Project issues: (your GitHub repo)

---

**Last Updated**: December 2025
**Docusaurus Version**: 3.x
**Node Version**: 18+
