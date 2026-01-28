import React, { useMemo } from 'react';
import { loadDefaultJapaneseParser } from 'budoux';
import { useLanguage } from '../contexts/LanguageContext';

type JapaneseLineBreakProps = {
  text: string;
  className?: string;
};

/**
 * Render Japanese text with smarter break opportunities using BudouX.
 * - Only active when current language is `ja`
 * - Splits text into semantic chunks and inserts <wbr/> between them.
 */
const JapaneseLineBreak: React.FC<JapaneseLineBreakProps> = ({ text, className }) => {
  const { language } = useLanguage();

  const nodes = useMemo(() => {
    if (language !== 'ja') return null;
    if (!text) return null;

    const parser = loadDefaultJapaneseParser();
    const chunks = parser.parse(text);

    if (!chunks || chunks.length === 0) {
      return [text];
    }

    return chunks.flatMap((chunk, idx) =>
      idx === 0 ? [chunk] : [<wbr key={`wbr-${idx}`} />, chunk]
    );
  }, [language, text]);

  if (language !== 'ja') {
    return <>{text}</>;
  }

  return (
    <span className={['ja-linebreak', className].filter(Boolean).join(' ')}>
      {nodes ?? text}
    </span>
  );
};

export default JapaneseLineBreak;

