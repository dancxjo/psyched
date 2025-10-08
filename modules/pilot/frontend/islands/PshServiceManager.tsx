import { useEffect, useState } from "preact/hooks";

import { Badge, type BadgeTone, Card } from "@pilot/components/dashboard.tsx";

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

type Message = {
  kind: "success" | "error";
  text: string;
};

const SERVICE_TONES: Record<ServiceStatus["status"], BadgeTone> = {
  running: "ok",
  stopped: "warn",
  error: "danger",
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
  const [message, setMessage] = useState<Message | null>(null);

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
        setMessage({ kind: "error", text: res.error ?? "Action failed" });
      } else {
        setMessage({ kind: "success", text: "Action completed successfully" });
        await refresh();
      }
    } catch (error) {
      setMessage({ kind: "error", text: String(error) });
    } finally {
      setLoading(false);
    }
  }

  return (
    <Card
      title="Service lifecycle"
      subtitle="Manage Docker compose stacks with psh srv"
      tone="magenta"
    >
      {message && (
        <p
          class={`note ${
            message.kind === "success" ? "note--success" : "note--alert"
          }`}
        >
          {message.text}
        </p>
      )}
      <div class="button-group button-group--wrap">
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/setup")}
        >
          Setup all
        </button>
        <button
          class="button button--danger"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/teardown")}
        >
          Teardown all
        </button>
        <button
          class="button button--primary"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/up")}
        >
          Start all
        </button>
        <button
          class="button button--ghost"
          type="button"
          disabled={loading}
          onClick={() => runAction("/api/psh/srv/down")}
        >
          Stop all
        </button>
      </div>
      <div class="table-scroll">
        <table class="data-table">
          <thead>
            <tr>
              <th scope="col">Service</th>
              <th scope="col">Status</th>
              <th scope="col">Description</th>
              <th scope="col">Actions</th>
            </tr>
          </thead>
          <tbody>
            {statuses.map((status) => (
              <tr key={status.name}>
                <td>{status.name}</td>
                <td>
                  <div class="status-indicator">
                    <Badge
                      label={status.status === "running"
                        ? "Running"
                        : status.status === "stopped"
                        ? "Stopped"
                        : "Error"}
                      tone={SERVICE_TONES[status.status]}
                    />
                  </div>
                </td>
                <td>{status.description ?? "—"}</td>
                <td>
                  <div class="button-group button-group--wrap">
                    <button
                      class="button button--small button--primary"
                      type="button"
                      disabled={loading}
                      onClick={() =>
                        runAction("/api/psh/srv/setup", [status.name])}
                    >
                      Setup
                    </button>
                    <button
                      class="button button--small button--primary"
                      type="button"
                      disabled={loading}
                      onClick={() =>
                        runAction("/api/psh/srv/up", [status.name])}
                    >
                      Start
                    </button>
                    <button
                      class="button button--small button--ghost"
                      type="button"
                      disabled={loading}
                      onClick={() =>
                        runAction("/api/psh/srv/down", [status.name])}
                    >
                      Stop
                    </button>
                    <button
                      class="button button--small button--danger"
                      type="button"
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
      </div>
    </Card>
  );
}
