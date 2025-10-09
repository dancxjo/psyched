import { useEffect, useRef } from "preact/hooks";
import type { ComponentChildren } from "preact";

import {
  OVERLAY_ORDER_STORAGE_KEY,
  parseOverlayOrderJson,
  reconcileOverlayOrder,
  serialiseOverlayOrder,
} from "../lib/dashboard/persisted_order.ts";

export interface OverlayGridProps {
  /**
   * Optional storage key override. Defaults to {@link OVERLAY_ORDER_STORAGE_KEY}.
   */
  storageKey?: string;
  children: ComponentChildren;
}

/**
 * Flexbox grid that supports drag-and-drop ordering for cockpit overlays.
 *
 * The component hydrates on the client so operators can rearrange dashboard
 * tiles. Positions are persisted in {@link localStorage} which allows the grid
 * to survive reloads on the same device.
 */
export default function OverlayGrid({
  storageKey = OVERLAY_ORDER_STORAGE_KEY,
  children,
}: OverlayGridProps) {
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (typeof window === "undefined") return;

    const container = containerRef.current;
    if (!container) return;

    const storageNamespace = `${storageKey}:${window.location.host ?? "localhost"}`;

    const ensureDraggable = (item: HTMLElement) => {
      item.setAttribute("draggable", "true");
      item.dataset.draggable = "true";
    };

    const attachListeners = (item: HTMLElement) => {
      ensureDraggable(item);
      item.addEventListener("dragstart", handleDragStart);
      item.addEventListener("dragend", handleDragEnd);
    };

    let activeDrag: HTMLElement | null = null;

    const detachListeners = (item: HTMLElement) => {
      item.removeEventListener("dragstart", handleDragStart);
      item.removeEventListener("dragend", handleDragEnd);
      item.removeAttribute("draggable");
      delete item.dataset.draggable;
      item.classList.remove("overlay-grid__item--dragging");
      if (activeDrag === item) {
        activeDrag = null;
      }
    };

    const commitOrder = () => {
      const order = Array.from(container.children)
        .map((child) => (child as HTMLElement).dataset.overlayKey)
        .filter((key): key is string => Boolean(key));
      window.localStorage.setItem(storageNamespace, serialiseOverlayOrder(order));
    };

    const applyStoredOrder = () => {
      const elements = Array.from(container.children) as HTMLElement[];
      const availableKeys = elements
        .map((element) => element.dataset.overlayKey)
        .filter((key): key is string => Boolean(key));
      const storedKeys = parseOverlayOrderJson(
        window.localStorage.getItem(storageNamespace),
      );
      const desiredOrder = reconcileOverlayOrder(availableKeys, storedKeys);
      const lookup = new Map<string, HTMLElement>();
      for (const element of elements) {
        const key = element.dataset.overlayKey;
        if (key) lookup.set(key, element);
      }
      for (const key of desiredOrder) {
        const element = lookup.get(key);
        if (element) {
          container.appendChild(element);
        }
      }
    };

    const handleDragStart = (event: DragEvent) => {
      const target = event.currentTarget as HTMLElement | null;
      if (!target) return;
      activeDrag = target;
      target.classList.add("overlay-grid__item--dragging");
      if (event.dataTransfer) {
        event.dataTransfer.effectAllowed = "move";
        const key = target.dataset.overlayKey ?? "";
        event.dataTransfer.setData("text/plain", key);
      }
    };

    const handleDragEnd = () => {
      if (!activeDrag) return;
      activeDrag.classList.remove("overlay-grid__item--dragging");
      activeDrag = null;
      commitOrder();
    };

    const handleDragOver = (event: DragEvent) => {
      if (!activeDrag) return;
      event.preventDefault();
      const target = (event.target as HTMLElement | null)?.closest<HTMLElement>(
        "[data-overlay-key]",
      );
      if (!target || target === activeDrag || target.parentElement !== container) {
        return;
      }
      const targetRect = target.getBoundingClientRect();
      const centerX = targetRect.left + targetRect.width / 2;
      const centerY = targetRect.top + targetRect.height / 2;
      const offsetX = event.clientX - centerX;
      const offsetY = event.clientY - centerY;
      const prioritizeHorizontal = Math.abs(offsetX) > Math.abs(offsetY);
      const before = prioritizeHorizontal ? offsetX < 0 : offsetY < 0;
      const referenceNode = before ? target : target.nextElementSibling;
      if (referenceNode !== activeDrag) {
        container.insertBefore(activeDrag, referenceNode ?? null);
      }
    };

    const handleDrop = (event: DragEvent) => {
      event.preventDefault();
      commitOrder();
    };

    const initialItems = Array.from(container.children) as HTMLElement[];
    for (const item of initialItems) {
      attachListeners(item);
    }
    applyStoredOrder();

    container.addEventListener("dragover", handleDragOver);
    container.addEventListener("drop", handleDrop);

    const observer = new MutationObserver((mutations) => {
      for (const mutation of mutations) {
        for (const node of mutation.addedNodes) {
          if (node instanceof HTMLElement) {
            attachListeners(node);
          }
        }
        for (const node of mutation.removedNodes) {
          if (node instanceof HTMLElement) {
            detachListeners(node);
          }
        }
      }
      applyStoredOrder();
    });
    observer.observe(container, { childList: true });

    return () => {
      observer.disconnect();
      const items = Array.from(container.children) as HTMLElement[];
      for (const item of items) {
        detachListeners(item);
      }
      container.removeEventListener("dragover", handleDragOver);
      container.removeEventListener("drop", handleDrop);
    };
  }, [storageKey]);

  return (
    <div ref={containerRef} class="overlay-grid">
      {children}
    </div>
  );
}
