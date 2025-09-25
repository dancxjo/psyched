import { build } from 'esbuild';
import sveltePlugin from 'esbuild-svelte';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = join(__dirname, '..');
const outDir = join(projectRoot, '..', 'static');

await build({
  entryPoints: [join(projectRoot, 'src', 'main.js')],
  bundle: true,
  format: 'esm',
  outfile: join(outDir, 'app.js'),
  plugins: [sveltePlugin({})],
  loader: { '.svg': 'file', '.png': 'file' },
  sourcemap: false,
  minify: true,
  target: ['es2020'],
  logLevel: 'info',
  write: true,
});
