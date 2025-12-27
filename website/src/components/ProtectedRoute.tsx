import React, { ReactNode } from 'react';
import { useAuth } from '../context/AuthContext';
import { Redirect } from '@docusaurus/router'; // Docusaurus router for redirects
import { useLocation } from '@docusaurus/router';

interface ProtectedRouteProps {
  children: ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({ children }) => {
  const location = useLocation();

  // Check if we're in a browser environment (not during static generation)
  const isBrowser = typeof window !== 'undefined';

  if (!isBrowser) {
    // During static generation, just return the children
    return <>{children}</>;
  }

  try {
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
  } catch (error) {
    // If there's an error accessing the auth context (e.g., during static generation),
    // redirect to login page
    return <Redirect to="/auth/login" />;
  }
};

export default ProtectedRoute;
