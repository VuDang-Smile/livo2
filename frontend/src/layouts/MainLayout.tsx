import React from 'react';
import SideMenu from '../components/SideMenu';
import LanguageSwitcher from '../components/LanguageSwitcher';

interface MainLayoutProps {
  children: React.ReactNode;
}

const MainLayout: React.FC<MainLayoutProps> = ({ children }) => {
  return (
    <div className="min-h-screen bg-gray-50 flex">
      {/* Side Menu */}
      <SideMenu />
      
      {/* Main Content Area */}
      <div className="flex-1 ml-64 min-w-0 w-full">
        {/* Header */}
        <header className="bg-white shadow-sm border-b border-gray-200">
          <div className="px-6 py-4">
            <div className="flex justify-end items-center">
              {/* Language Switcher */}
              <div className="flex items-center space-x-4">
                <LanguageSwitcher />
              </div>
            </div>
          </div>
        </header>

        {/* Main content */}
        <main className="p-6">
          {children}
        </main>
      </div>
    </div>
  );
};

export default MainLayout;