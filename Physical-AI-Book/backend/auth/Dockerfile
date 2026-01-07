# Auth Service Dockerfile - Railway Compatible
FROM node:20-alpine

WORKDIR /app

# Copy package files
COPY package*.json ./
COPY tsconfig.json ./
COPY package-lock.json ./

# Install all dependencies (including devDependencies needed for tsx runtime)
RUN npm ci

# Copy source code
COPY src ./src

# Railway will set the PORT environment variable dynamically
# Don't expose a specific port - Railway handles this automatically

# Health check using dynamic PORT environment variable
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD node -e "require('http').get('http://localhost:' + (process.env.PORT || 3001) + '/health', (r) => {process.exit(r.statusCode === 200 ? 0 : 1)})"

# Start server (reads PORT from Railway environment)
CMD ["npm", "start"]
