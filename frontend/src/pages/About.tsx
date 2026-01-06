import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';

const About: React.FC = () => {
  const { t } = useLanguage();
  return (
    <div className="space-y-16">
      {/* Hero Section */}
      <section className="text-center">
        <h1 className="text-4xl font-bold text-gray-900 mb-6">
          {t('about_hero_title')}
        </h1>
        <p className="text-xl text-gray-600 max-w-4xl mx-auto">
          {t('about_hero_desc')}
        </p>
      </section>

      {/* Mission Section */}
      <section className="bg-white rounded-2xl p-8 shadow-sm border border-gray-200">
        <div className="grid md:grid-cols-2 gap-8 items-center">
          <div>
            <h2 className="text-3xl font-bold text-gray-900 mb-4">
              {t('about_mission_title')}
            </h2>
            <p className="text-gray-600 mb-6">
              {t('about_mission_desc')}
            </p>
            <div className="space-y-3">
              <div className="flex items-center">
                <div className="w-2 h-2 bg-primary-600 rounded-full mr-3"></div>
                <span className="text-gray-700">{t('about_mission_1')}</span>
              </div>
              <div className="flex items-center">
                <div className="w-2 h-2 bg-primary-600 rounded-full mr-3"></div>
                <span className="text-gray-700">{t('about_mission_2')}</span>
              </div>
              <div className="flex items-center">
                <div className="w-2 h-2 bg-primary-600 rounded-full mr-3"></div>
                <span className="text-gray-700">{t('about_mission_3')}</span>
              </div>
            </div>
          </div>
          <div className="text-center">
            <div className="text-8xl mb-4">🏗️</div>
            <h3 className="text-xl font-semibold text-gray-900">
              {t('about_mission_safety')}
            </h3>
          </div>
        </div>
      </section>
    </div>
  );
};

export default About; 