import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../../context/AuthContext';

const LoginPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [error, setError] = useState<string | null>(null);
  const { login } = useAuth();

  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      const response = await fetch('http://localhost:3001/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email }),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.message || 'Login failed');
      }

      const data = await response.json();
      login(data.token);
      window.location.href = '/'; // Redirect to home or dashboard
    } catch (err: any) {
      setError(err.message);
    }
  };

  return (
    <Layout title="Login" description="Login to your account">
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="text--center">Login</h1>
            <form onSubmit={handleLogin} className="card padding--lg">
              <div className="margin-bottom--md">
                <label htmlFor="email" className="form__label">Email</label>
                <input
                  type="email"
                  id="email"
                  className="form__input"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>
              {error && <div className="alert alert--danger margin-bottom--md">{error}</div>}
              <button type="submit" className="button button--primary button--block">Login</button>
              <p className="text--center margin-top--md">
                Don't have an account? <a href="/auth/signup">Sign Up</a>
              </p>
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default LoginPage;
