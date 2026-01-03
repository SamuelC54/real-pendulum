/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'bg-dark': '#0a0a0f',
        'bg-panel': '#12121a',
        'accent-cyan': '#00d4ff',
        'accent-magenta': '#ff00aa',
        'accent-orange': '#ff6b35',
        'accent-green': '#00ff88',
        'text-primary': '#e0e0e0',
        'text-dim': '#666680',
        'pendulum-rod': '#4a9eff',
        'pendulum-bob': '#ff6b35',
        'pivot': '#00d4ff',
        'neat-purple': '#c864ff',
      },
      fontFamily: {
        'sans': ['Space Grotesk', 'sans-serif'],
        'mono': ['JetBrains Mono', 'monospace'],
      },
      backgroundImage: {
        'gradient-radial': 'radial-gradient(var(--tw-gradient-stops))',
      },
    },
  },
  plugins: [],
}

