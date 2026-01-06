import React, { useState, useRef } from 'react';
import QRCode from 'react-qr-code';
import { useLanguage } from '../contexts/LanguageContext';

const QRGenerator: React.FC = () => {
  const { t } = useLanguage();
  const [index, setIndex] = useState<string>('');
  const [qrValue, setQrValue] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const qrRef = useRef<HTMLDivElement>(null);

  const validateIndex = (value: string): boolean => {
    const num = parseInt(value, 10);
    return !isNaN(num) && num >= 0 && num <= 999;
  };

  const formatIndex = (value: string): string => {
    const num = parseInt(value, 10);
    if (isNaN(num)) return '';
    return num.toString().padStart(3, '0');
  };

  const handleIndexChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = e.target.value;
    setIndex(value);
    setError(null);
  };

  const handleCreateQR = () => {
    if (!index.trim()) {
      setError(t('qr_generator_index_invalid'));
      return;
    }

    if (!validateIndex(index)) {
      setError(t('qr_generator_index_invalid'));
      return;
    }

    const formattedIndex = formatIndex(index);
    const qrContent = `TM:${formattedIndex}`;
    setQrValue(qrContent);
    setError(null);
  };

  const handleDownloadPNG = async () => {
    if (!qrValue || !qrRef.current) return;
    const currentQrValue = qrValue;

    try {
      // Get the SVG element from the QR code
      const svgElement = qrRef.current.querySelector('svg');
      if (!svgElement) return;

      // Convert SVG to canvas
      const svgData = new XMLSerializer().serializeToString(svgElement);
      const canvas = document.createElement('canvas');
      const ctx = canvas.getContext('2d');
      const img = new Image();

      const svgBlob = new Blob([svgData], { type: 'image/svg+xml;charset=utf-8' });
      const svgUrl = URL.createObjectURL(svgBlob);
      img.src = svgUrl;
      img.onload = () => {
        canvas.width = img.width;
        canvas.height = img.height;
        if (ctx) {
          ctx.fillStyle = 'white';
          ctx.fillRect(0, 0, canvas.width, canvas.height);
          ctx.drawImage(img, 0, 0);

          // Download as PNG
          canvas.toBlob((blob) => {
            if (blob) {
              const url = URL.createObjectURL(blob);
              const link = document.createElement('a');
              const formattedIndex = currentQrValue.replace('TM:', '');
              link.download = `TM-${formattedIndex}.png`;
              link.href = url;
              link.click();
              URL.revokeObjectURL(url);
            }
          }, 'image/png');
        }
        URL.revokeObjectURL(svgUrl);
      };

    } catch (err) {
      console.error('Error downloading QR code:', err);
    }
  };

  const handleCopyClipboard = async () => {
    if (!qrValue || !qrRef.current) return;

    try {
      // Get the SVG element
      const svgElement = qrRef.current.querySelector('svg');
      if (!svgElement) return;

      // Convert SVG to blob
      const svgData = new XMLSerializer().serializeToString(svgElement);
      const svgBlob = new Blob([svgData], { type: 'image/svg+xml;charset=utf-8' });
      const svgUrl = URL.createObjectURL(svgBlob);
      
      // Convert to PNG blob
      const img = new Image();
      const canvas = document.createElement('canvas');
      const ctx = canvas.getContext('2d');

      img.onload = () => {
        canvas.width = img.width;
        canvas.height = img.height;
        if (ctx) {
          ctx.fillStyle = 'white';
          ctx.fillRect(0, 0, canvas.width, canvas.height);
          ctx.drawImage(img, 0, 0);

          canvas.toBlob(async (blob) => {
            if (blob) {
              try {
                await navigator.clipboard.write([
                  new ClipboardItem({ 'image/png': blob })
                ]);
                alert(t('qr_generator_copy_success'));
              } catch (err) {
                console.error('Error copying to clipboard:', err);
                alert(t('qr_generator_copy_error'));
              }
            }
          }, 'image/png');
        }
        URL.revokeObjectURL(svgUrl);
      };

      img.src = svgUrl;
    } catch (err) {
      console.error('Error copying QR code:', err);
      alert(t('qr_generator_copy_error'));
    }
  };

  const handleReset = () => {
    setIndex('');
    setQrValue(null);
    setError(null);
  };

  return (
    <div className="space-y-6">
      {/* Page header */}
      <div className="bg-white p-6 rounded-xl shadow-sm border border-gray-200">
        <h1 className="text-3xl font-bold text-gray-900 mb-2">
          {t('qr_generator_title')}
        </h1>
      </div>

      <div className="grid gap-6 lg:grid-cols-[1fr,1.2fr]">
        <div className="bg-white p-6 rounded-xl shadow-sm border border-gray-200">
          <div className="space-y-4">
            <div>
              <label htmlFor="index" className="block text-xl font-semibold text-gray-900 mb-2">
                {t('qr_generator_index_label')}
              </label>
              <input
                type="text"
                inputMode="numeric"
                pattern="[0-9]*"
                id="index"
                value={index}
                onChange={handleIndexChange}
                placeholder={t('qr_generator_index_placeholder')}
                className={`w-full px-3 py-2 border rounded-md shadow-sm focus:outline-none focus:ring-primary-500 focus:border-primary-500 ${
                  error ? 'border-red-500' : 'border-gray-300'
                }`}
                maxLength={3}
              />
              {error && (
                <p className="mt-1 text-sm text-red-600">{error}</p>
              )}
              <p className="mt-1 text-xs text-gray-500">
                {t('qr_generator_index_placeholder')}
              </p>
            </div>

            <button
              onClick={handleCreateQR}
              className="inline-flex items-center px-4 py-2 bg-primary-600 text-white text-sm font-medium rounded-md shadow-sm hover:bg-primary-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-primary-500"
            >
              {t('qr_generator_create_button')}
            </button>
          </div>
        </div>

        <div className="bg-white p-6 rounded-xl shadow-sm border border-gray-200">
          <h2 className="text-xl font-semibold text-gray-900 mb-4">
            {t('qr_generator_preview_title')}
          </h2>
          <div className="space-y-4">
            {qrValue ? (
              <>
                <div className="bg-gray-50 p-3 rounded-lg">
                  <p className="text-sm text-gray-700">
                    <span className="font-medium">{t('qr_generator_qr_content')}:</span>{' '}
                    <span className="font-mono">{qrValue}</span>
                  </p>
                </div>

                <div className="flex justify-center p-6 bg-white border border-gray-200 rounded-lg">
                  <div ref={qrRef} className="bg-white p-4 rounded-lg">
                    <QRCode
                      value={qrValue}
                      size={256}
                      level="H"
                      bgColor="#FFFFFF"
                      fgColor="#000000"
                    />
                  </div>
                </div>

                <div className="flex flex-wrap gap-3">
                  <button
                    onClick={handleDownloadPNG}
                    className="inline-flex items-center px-4 py-2 bg-blue-600 text-white text-sm font-medium rounded-md shadow-sm hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
                  >
                    <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 16v1a3 3 0 003 3h10a3 3 0 003-3v-1m-4-4l-4 4m0 0l-4-4m4 4V4" />
                    </svg>
                    {t('qr_generator_download_png')}
                  </button>

                  <button
                    onClick={handleCopyClipboard}
                    className="inline-flex items-center px-4 py-2 bg-green-600 text-white text-sm font-medium rounded-md shadow-sm hover:bg-green-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-green-500"
                  >
                    <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 16H6a2 2 0 01-2-2V6a2 2 0 012-2h8a2 2 0 012 2v2m-6 12h8a2 2 0 002-2v-8a2 2 0 00-2-2h-8a2 2 0 00-2 2v8a2 2 0 002 2z" />
                    </svg>
                    {t('qr_generator_copy_clipboard')}
                  </button>

                  <button
                    onClick={handleReset}
                    className="inline-flex items-center px-4 py-2 bg-gray-600 text-white text-sm font-medium rounded-md shadow-sm hover:bg-gray-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-gray-500"
                  >
                    <svg className="w-5 h-5 mr-2" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                    </svg>
                    {t('qr_generator_reset')}
                  </button>
                </div>
              </>
            ) : (
              <div className="flex flex-col items-center justify-center min-h-[320px] border border-dashed border-gray-200 rounded-lg bg-white text-gray-500">
                <div className="text-gray-400 mb-4">
                  <svg className="mx-auto h-16 w-16" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 4v1m6 11h2m-6 0h-2v4m0-11v3m0 0h.01M12 12h4.01M16 20h4M4 12h4m12 0h.01M5 8h2a1 1 0 001-1V5a1 1 0 00-1-1H5a1 1 0 00-1 1v2a1 1 0 001 1zm12 0h2a1 1 0 001-1V5a1 1 0 00-1-1h-2a1 1 0 00-1 1v2a1 1 0 001 1zM5 20h2a1 1 0 001-1v-2a1 1 0 00-1-1H5a1 1 0 00-1 1v2a1 1 0 001 1z" />
                  </svg>
                </div>
                <p className="text-sm text-center px-4">
                  {t('qr_generator_no_qr')}
                </p>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default QRGenerator;

