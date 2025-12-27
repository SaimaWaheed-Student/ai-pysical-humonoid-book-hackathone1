import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../../context/AuthContext';
import ProtectedRoute from '../../components/ProtectedRoute'; // Adjust path as necessary

interface Task {
  jobId: string;
  name: string;
  state: string;
  progress: number;
  result: any;
  failedReason?: string;
  data: any;
  createdAt: string;
  finishedAt: string | null;
}

const TaskDashboard: React.FC = () => {
  // Use a try-catch to safely access the auth context
  let auth;
  try {
    auth = useAuth();
  } catch (err) {
    // If context is not available (during static generation), set auth to null
    auth = null;
  }

  const [tasks, setTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchTasks = async () => {
      if (!auth || !auth.token) {
        setLoading(false);
        return;
      }

      try {
        // This would ideally be a /api/tasks endpoint that lists all tasks for the user
        // For now, we don't have a list all tasks endpoint, so this is a placeholder.
        // It will fetch a specific task if we hardcode an ID, or just show empty.
        // Once the DB stores tasks, this can be fleshed out.
        // fetch(`${process.env.BACKEND_URL}/api/tasks`, { headers: { Authorization: `Bearer ${auth.token}` }})
        // For now, it's just a UI placeholder.
        setTasks([]); // No tasks to display yet without a proper listing endpoint
      } catch (err: any) {
        setError(err.message || 'Failed to fetch tasks');
      } finally {
        setLoading(false);
      }
    };

    fetchTasks();
  }, [auth]);

  // If auth context is not available during static generation, return a placeholder
  if (!auth) {
    return (
      <Layout title="Task Dashboard" description="Manage and monitor background tasks">
        <main className="container margin-vert--xl">
          <div className="text--center">Loading authentication system...</div>
        </main>
      </Layout>
    );
  }

  if (loading) {
    return (
      <Layout title="Task Dashboard">
        <main className="container margin-vert--xl">
          <h1>Loading Tasks...</h1>
        </main>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Task Dashboard">
        <main className="container margin-vert--xl">
          <div className="alert alert--danger">{error}</div>
        </main>
      </Layout>
    );
  }

  return (
    <ProtectedRoute>
      <Layout title="Task Dashboard" description="Manage and monitor background tasks">
        <main className="container margin-vert--xl">
          <h1 className="text--center">Task Dashboard</h1>
          {tasks.length === 0 ? (
            <p className="text--center">No tasks found. Create a task to see it here.</p>
          ) : (
            <div>
              {/* Render tasks here */}
              {tasks.map(task => (
                <div key={task.jobId} className="card margin-bottom--md">
                  <div className="card__header">
                    <h3>{task.name} ({task.jobId})</h3>
                  </div>
                  <div className="card__body">
                    <p><strong>Status:</strong> {task.state}</p>
                    <p><strong>Progress:</strong> {task.progress}%</p>
                    {task.failedReason && <p className="text--danger"><strong>Error:</strong> {task.failedReason}</p>}
                    <p><strong>Created:</strong> {new Date(task.createdAt).toLocaleString()}</p>
                    {task.finishedAt && <p><strong>Finished:</strong> {new Date(task.finishedAt).toLocaleString()}</p>}
                  </div>
                </div>
              ))}
            </div>
          )}
        </main>
      </Layout>
    </ProtectedRoute>
  );
};

export default TaskDashboard;
