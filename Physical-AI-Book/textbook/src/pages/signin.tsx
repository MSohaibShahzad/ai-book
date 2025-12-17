import React from 'react';
import Layout from '@theme/Layout';
import { SigninForm } from '../components/auth/SigninForm';

export default function SigninPage() {
  return (
    <Layout
      title="Sign In"
      description="Sign in to your Physical-AI account"
    >
      <div style={{ padding: '2rem 0' }}>
        <SigninForm />
      </div>
    </Layout>
  );
}
