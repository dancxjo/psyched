import { useEffect, useState } from "preact/hooks";

type ServiceStatus = {
  name: string;
  status: "running" | "stopped" | "error";
  description?: string;
};

type ApiResponse = {
  ok: boolean;
  statuses?: ServiceStatus[];
  error?: string;
};

type ActionResponse = {
  ok: boolean;
  error?: string;
};

async function post(url: string, body?: unknown) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });
  return response.json();
}

export default function PshServiceManager() {
  const [statuses, setStatuses] = useState<ServiceStatus[]>([]);
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState<string | null>(null);

  async function refresh() {
    const res: ApiResponse = await fetch("/api/psh/srv/list").then((r) =>
      r.json()
    );
    if (res.ok && res.statuses) {
      setStatuses(res.statuses);
    }
  }

  useEffect(() => {
    refresh();
  }, []);

  async function runAction(endpoint: string, services?: string[]) {
    setLoading(true);
    setMessage(null);
    try {
      const res: ActionResponse = await post(
        endpoint,
        services?.length ? { services } : undefined,
      );
      if (!res.ok) {
        setMessage(res.error ?? "Action failed");
      } else {
        setMessage("Action completed successfully");
        await refresh();
      }
    } catch (error) {
      setMessage(String(error));
    } finally {
      setLoading(false);
    }
  }

  return (
    <section class="psh-card">
      <header>
        <h2>Service Lifecycle</h2>
        <p>
          Manage Docker compose stacks via <code>psh srv</code>.
        </p>
      </header>
      {message && <p class="psh-message">{message}</p>}
      <div class="psh-actions">
        <button
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/setup")}
        >
          Setup All
        </button>
        <button
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/teardown")}
        >
          Teardown All
        </button>
        <button disabled={loading} onClick={() => runAction("/api/psh/srv/up")}>
          Start All
        </button>
        <button
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/down")}
        >
          Stop All
        </button>
      </div>
      <table class="psh-table">
        <thead>
          <tr>
            <th>Service</th>
            <th>Status</th>
            <th>Description</th>
            <th>Actions</th>
          </tr>
        </thead>
        <tbody>
          {statuses.map((status) => (
            <tr key={status.name}>
              <td>{status.name}</td>
              <td>{status.status}</td>
              <td>{status.description ?? "—"}</td>
              <td>
                <div class="psh-row-actions">
                  <button
                    disabled={loading}
                    onClick={() =>
                      runAction("/api/psh/srv/setup", [status.name])}
                  >
                    Setup
                  </button>
                  <button
                    disabled={loading}
                    onClick={() => runAction("/api/psh/srv/up", [status.name])}
                  >
                    Start
                  </button>
                  <button
                    disabled={loading}
                    onClick={() =>
                      runAction("/api/psh/srv/down", [status.name])}
                  >
                    Stop
                  </button>
                  <button
                    disabled={loading}
                    onClick={() =>
                      runAction("/api/psh/srv/teardown", [status.name])}
                  >
                    Teardown
                  </button>
                </div>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}
