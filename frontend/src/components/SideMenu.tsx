import React from 'react';
import { Link, useLocation } from 'react-router-dom';
import { useLanguage } from '../contexts/LanguageContext';
import { Info, Map, Car, Upload, QrCode, Heart } from 'lucide-react';

const SideMenu: React.FC = () => {
  const location = useLocation();
  const { t } = useLanguage();

  const navigation = [
    { name: t('about'), href: '/about', icon: Info },
    { name: t('map'), href: '/vehicle-map', icon: Map },
    { name: t('vehicles'), href: '/vehicles', icon: Car },
    { name: t('upload_menu_label'), href: '/map-upload', icon: Upload },
    { name: t('qr_generator_menu_label'), href: '/qr-generator', icon: QrCode },
    { name: t('health_menu_label'), href: '/health', icon: Heart },
  ];

  return (
    <div className="w-64 bg-white shadow-lg border-r border-gray-200 fixed left-0 top-0 h-full z-50 min-h-screen">
      {/* Logo */}
      <div className="p-4 border-b border-gray-200">
        <Link to="/" className="flex items-center space-x-2">
          <img src="/logo.png" alt="Tunnel & Mining Logo" className="w-full h-full object-contain" />
        </Link>
      </div>

      {/* Navigation Menu */}
      <nav className="mt-4">
        <div className="px-3">
          <ul className="space-y-1">
            {navigation.map((item) => (
              <li key={item.name}>
                <Link
                  to={item.href}
                  className={`flex items-center px-3 py-2 text-xs font-medium rounded-md transition-all duration-200 ${
                    location.pathname === item.href
                      ? 'bg-primary-50 text-primary-700 border-r-2 border-primary-600'
                      : 'text-gray-600 hover:bg-gray-50 hover:text-gray-900'
                  }`}
                >
                  <item.icon className="mr-2 h-4 w-4" />
                  {item.name}
                </Link>
              </li>
            ))}
          </ul>
        </div>
      </nav>
    </div>
  );
};

export default SideMenu; 