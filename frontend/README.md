# Layout Design - ReactJS Project

Dự án ReactJS hoàn chỉnh với Tailwind CSS, React Router và light theme.

## 🚀 Tính năng

- **React 19** với TypeScript
- **Tailwind CSS** cho styling
- **React Router** cho navigation
- **Light Theme** được thiết kế sẵn
- **Responsive Design** cho mọi thiết bị
- **Modern UI/UX** với animations và transitions

## 📁 Cấu trúc dự án

```
src/
├── components/          # Reusable components
├── layouts/            # Layout components
│   └── MainLayout.tsx  # Main layout với navigation
├── pages/              # Page components
│   ├── Home.tsx        # Trang chủ
│   ├── About.tsx       # Trang giới thiệu
│   ├── Projects.tsx    # Trang dự án
│   └── Contact.tsx     # Trang liên hệ
├── App.tsx             # Main App component
└── index.tsx           # Entry point
```

## 🛠️ Cài đặt

1. **Clone dự án:**
   ```bash
   git clone <repository-url>
   cd layout_design
   ```

2. **Cài đặt dependencies:**
   ```bash
   npm install
   ```

3. **Chạy development server:**
   ```bash
   npm start
   ```

4. **Build cho production:**
   ```bash
   npm run build
   ```

## 🎨 Công nghệ sử dụng

- **React 19** - UI Library
- **TypeScript** - Type safety
- **Tailwind CSS** - Utility-first CSS framework
- **React Router** - Client-side routing
- **PostCSS** - CSS processing
- **Autoprefixer** - CSS vendor prefixes

## 📱 Pages

### 🏠 Trang chủ (Home)
- Hero section với call-to-action
- Features showcase
- Modern design với gradients

### ℹ️ Giới thiệu (About)
- Thông tin về team
- Công nghệ sử dụng
- Statistics và achievements

### 📁 Dự án (Projects)
- Portfolio projects với filtering
- Project categories (Web, Mobile, UI/UX)
- Project status indicators

### 📧 Liên hệ (Contact)
- Contact form với validation
- Contact information
- Social media links
- FAQ section

## 🎯 Tính năng nổi bật

- **Light Theme**: Thiết kế sáng với màu sắc dễ chịu
- **Responsive**: Tương thích mọi thiết bị
- **Navigation**: Smooth navigation với active states
- **Forms**: Form validation và user feedback
- **Animations**: Smooth transitions và hover effects
- **Accessibility**: Semantic HTML và keyboard navigation

## 🔧 Tùy chỉnh

### Thay đổi màu sắc
Chỉnh sửa file `tailwind.config.js`:
```javascript
theme: {
  extend: {
    colors: {
      primary: {
        50: '#eff6ff',
        // ... thêm các shades khác
      },
    },
  },
}
```

### Thêm trang mới
1. Tạo component trong `src/pages/`
2. Thêm route trong `src/App.tsx`
3. Thêm navigation item trong `src/layouts/MainLayout.tsx`

## 📦 Scripts

- `npm start` - Chạy development server
- `npm run build` - Build cho production
- `npm test` - Chạy tests
- `npm run eject` - Eject từ Create React App

## 🌟 Demo

Dự án bao gồm:
- ✅ 4 trang hoàn chỉnh
- ✅ Responsive design
- ✅ Modern UI/UX
- ✅ Form handling
- ✅ State management
- ✅ TypeScript support
- ✅ Tailwind CSS styling

## 📄 License

MIT License - Xem file LICENSE để biết thêm chi tiết.

## 🤝 Đóng góp

Mọi đóng góp đều được chào đón! Hãy tạo issue hoặc pull request.

---

**Layout Design** - Built with ❤️ using React & Tailwind CSS
