import React, { ReactNode } from 'react';
import { useAuth } from '../context/AuthContext';
import { Redirect } from '@docusaurus/router'; // Docusaurus router for redirects

interface ProtectedRouteProps {
  children: ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({ children }) => {
  const { user, isLoading } = useAuth();

  if (isLoading) {
    // Optionally render a loading spinner or placeholder
    return <div>Loading authentication...</div>; 
  }

  if (!user) {
    // Redirect unauthenticated users to the login page
    return <Redirect to="/auth/login" />;
  }

  return <>{children}</>;
};

export default ProtectedRoute;
