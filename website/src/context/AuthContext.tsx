import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

interface AuthContextType {
  user: { id: string; email: string; roles: string } | null;
  token: string | null;
  login: (token: string) => void;
  logout: () => void;
  isLoading: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<{ id: string; email: string; roles: string } | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true); // Loading state for initial token check

  useEffect(() => {
    // Attempt to load token from localStorage on initial render
    const storedToken = localStorage.getItem('authToken');
    if (storedToken) {
      setToken(storedToken);
      // In a real app, you'd verify the token with the backend and fetch user data
      // For now, we'll decode it locally (not secure for production) or trust it.
      // A more robust solution would involve fetching /api/auth/me
      // For this prototype, we'll assume the token is valid for the sake of progression
      try {
        const payload = JSON.parse(atob(storedToken.split('.')[1])); // Basic JWT decode
        setUser({ id: payload.id, email: payload.email, roles: payload.roles });
      } catch (e) {
        console.error("Failed to decode token from localStorage", e);
        localStorage.removeItem('authToken'); // Clear invalid token
      }
    }
    setIsLoading(false);
  }, []);

  const login = (newToken: string) => {
    localStorage.setItem('authToken', newToken);
    setToken(newToken);
    try {
      const payload = JSON.parse(atob(newToken.split('.')[1]));
      setUser({ id: payload.id, email: payload.email, roles: payload.roles });
    } catch (e) {
      console.error("Failed to decode new token", e);
    }
  };

  const logout = () => {
    localStorage.removeItem('authToken');
    setToken(null);
    setUser(null);
  };

  return (
    <AuthContext.Provider value={{ user, token, login, logout, isLoading }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
