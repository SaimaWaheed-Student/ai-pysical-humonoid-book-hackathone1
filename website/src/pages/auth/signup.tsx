import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../../context/AuthContext';

const SignupPage: React.FC = () => {
  const [email, setEmail] = useState('');
  const [error, setError] = useState<string | null>(null);
  const { login } = useAuth();

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      const response = await fetch('http://localhost:3001/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email }),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.message || 'Signup failed');
      }

      const data = await response.json();
      login(data.token);
      window.location.href = '/'; // Redirect to home or dashboard
    } catch (err: any) {
      setError(err.message);
    }
  };

  return (
    <Layout title="Sign Up" description="Create a new account">
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="text--center">Sign Up</h1>
            <form onSubmit={handleSignup} className="card padding--lg">
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
              <button type="submit" className="button button--primary button--block">Sign Up</button>
              <p className="text--center margin-top--md">
                Already have an account? <a href="/auth/login">Login</a>
              </p>
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default SignupPage;
