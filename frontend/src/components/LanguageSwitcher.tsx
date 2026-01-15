import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';

const LanguageSwitcher: React.FC = () => {
  const { language, setLanguage, t } = useLanguage();

  const handleLanguageChange = () => {
    // Cycle through vi -> ja -> en -> vi
    const newLanguage = language === 'vi' ? 'ja' : language === 'ja' ? 'en' : 'vi';
    setLanguage(newLanguage);
  };

  return (
    <button
      onClick={handleLanguageChange}
      className="w-full relative flex items-center justify-center px-3 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 transition-colors duration-200"
      title={t('switch_language')}
    >
      <div className="flex items-center">
        <span className="text-lg mr-2">
          {language === 'ja' ? '🇯🇵' : language === 'en' ? '🇺🇸' : '🇻🇳'}
        </span>
        <span className="hidden sm:inline">
          {language === 'ja' ? '日本語' : language === 'en' ? 'English' : 'Tiếng Việt'}
        </span>
      </div>
    </button>
  );
};

export default LanguageSwitcher; 