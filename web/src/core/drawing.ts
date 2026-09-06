/** Solvent Drawing binding. Parsing, solving, reference checks, and rendering stay in Rust. */
import { core, takeJson, withJson, withStr } from './wasm.js';

export interface DrawingBundle { source: string; files: Record<string, string> }

export interface DrawingInfo { models: string[]; imports: string[]; sheets: string[] }
export function info(text: string): DrawingInfo {
  const result = withStr(text, (p, n) => takeJson<DrawingInfo & { error?: string }>(core().gcs_drawing_info(p, n)));
  if (!result || result.error) throw new Error(result?.error ?? 'Could not read drawing');
  return result;
}

export function render(text: string, source: string, files: Record<string, string>, sheet?: string): string {
  const result = withJson({ text, source, files, sheet }, (p, n) =>
    takeJson<{ svg?: string; error?: string }>(core().gcs_drawing_svg(p, n)));
  if (!result?.svg) throw new Error(result?.error ?? 'Could not render drawing');
  return result.svg;
}
