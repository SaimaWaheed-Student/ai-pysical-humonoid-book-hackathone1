# Book AI Website

This is the frontend website for the Book AI project, built with Docusaurus.

## About

This website provides an interactive interface for users to interact with book content using AI. It's built with Docusaurus, a modern static website generator.

## Local Development

1. Navigate to the website directory:
```bash
cd website
```

2. Install dependencies:
```bash
npm install
```

3. Run the development server:
```bash
npm start
```

The website will be available at `http://localhost:3000`.

## Building for Production

To build the website for production:

```bash
cd website
npm run build
```

The built files will be in the `build` directory.

## Deployment

This website is configured for deployment to Vercel. When deploying to Vercel:

1. Select the `website` directory as the root directory
2. Vercel will automatically detect this as a Docusaurus project
3. The build command will be `npm run build`
4. The output directory will be `build`

## Features

- Interactive documentation for the book content
- Search functionality
- Responsive design
- Progressive Web App (PWA) capabilities