import React from 'react';
import Layout from '@theme/Layout';
import { ProfileSettings } from '../components/auth/ProfileSettings';

export default function ProfilePage() {
  return (
    <Layout
      title="Profile Settings"
      description="Manage your Physical-AI account settings"
    >
      <div style={{ padding: '2rem 0' }}>
        <ProfileSettings />
      </div>
    </Layout>
  );
}
