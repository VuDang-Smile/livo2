import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { LanguageProvider } from './contexts/LanguageContext';
import { AuthProvider } from './contexts/AuthContext';
import MainLayout from './layouts/MainLayout';
import ProtectedRoute from './components/ProtectedRoute';
import About from './pages/About';
import Login from './pages/Login';
import Vehicles from './pages/Vehicles';
import CreateVehicle from './pages/CreateVehicle';
import EditVehicle from './pages/EditVehicle';
import VehicleMap from './pages/VehicleMap';
import Upload from './pages/Upload';
import QRGenerator from './pages/QRGenerator';
import Health from './pages/Health';

function App() {
  return (
    <LanguageProvider>
      <AuthProvider>
        <Router>
          <Routes>
            <Route path="/login" element={<MainLayout><Login /></MainLayout>} />
            <Route path="/" element={<MainLayout><About /></MainLayout>} />
            <Route path="/about" element={<MainLayout><About /></MainLayout>} />
            <Route path="/vehicles" element={<MainLayout><ProtectedRoute><Vehicles /></ProtectedRoute></MainLayout>} />
            <Route path="/vehicles/create" element={<MainLayout><ProtectedRoute><CreateVehicle /></ProtectedRoute></MainLayout>} />
            <Route path="/vehicles/edit/:id" element={<MainLayout><ProtectedRoute><EditVehicle /></ProtectedRoute></MainLayout>} />
            <Route path="/vehicle-map" element={<MainLayout><VehicleMap /></MainLayout>} />
            <Route path="/map-upload" element={<MainLayout><ProtectedRoute><Upload /></ProtectedRoute></MainLayout>} />
            <Route path="/qr-generator" element={<MainLayout><ProtectedRoute><QRGenerator /></ProtectedRoute></MainLayout>} />
            <Route path="/health" element={<MainLayout><Health /></MainLayout>} />
          </Routes>
        </Router>
      </AuthProvider>
    </LanguageProvider>
  );
}

export default App;
