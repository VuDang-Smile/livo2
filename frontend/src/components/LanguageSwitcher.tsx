import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';

const LanguageSwitcher: React.FC = () => {
  const { language, setLanguage, t } = useLanguage();

  const handleLanguageChange = () => {
    const newLanguage = language === 'vi' ? 'ja' : 'vi';
    setLanguage(newLanguage);
  };

  return (
    <button
      onClick={handleLanguageChange}
      className="flex items-center space-x-2 px-3 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 transition-colors duration-200"
      title={t('switch_language')}
    >
      <span className="text-lg">
        {language === 'ja' ? '🇯🇵' : '🇻🇳'}
      </span>
      <span className="hidden sm:inline">
        {language === 'ja' ? '日本語' : 'Tiếng Việt'}
      </span>
    </button>
  );
};

export default LanguageSwitcher; 