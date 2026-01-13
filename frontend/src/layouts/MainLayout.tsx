import React from 'react';
import SideMenu from '../components/SideMenu';

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
        {/* Main content */}
        <main className="p-6">
          {children}
        </main>
      </div>
    </div>
  );
};

export default MainLayout;