/**
 * Swizzled DocItem/Layout component to add Translation Button.
 *
 * This component wraps the default Docusaurus DocItem layout and adds
 * the Urdu translation button at the top of each documentation page.
 */

import React from 'react';
import DocItemLayout from '@theme-original/DocItem/Layout';
import TranslationButtonEnhanced from '@site/src/components/TranslationButton/TranslationButtonEnhanced';
import type { Props} from '@theme/DocItem/Layout';
import { useDoc } from '@docusaurus/plugin-content-docs/client';

export default function DocItemLayoutWrapper(props: Props): JSX.Element {
  const { metadata } = useDoc();

  // Use the permalink from metadata, which is the actual URL path
  // e.g., "/docs/preface/how-to-use" → "preface/how-to-use"
  // This matches the URL structure WITHOUT numbers
  const chapterSlug = (metadata.permalink || '')
    .replace('/docs/', '')
    .replace(/^\//, '')
    .replace(/\/$/, '');

  return (
    <>
      <div style={{ marginBottom: '1.5rem' }}>
        <TranslationButtonEnhanced chapterSlug={chapterSlug} />
      </div>
      <DocItemLayout {...props} />
    </>
  );
}
