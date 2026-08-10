import { describe, expect, it } from '@jest/globals';

import { cameraBearingFromDeviceHeading } from '../orientation';

describe('cameraBearingFromDeviceHeading', () => {
  it('maps landscape-right device-top west to camera north', () => {
    expect(cameraBearingFromDeviceHeading(270, 90)).toBe(0);
  });

  it('maps landscape-left device-top east to camera north', () => {
    expect(cameraBearingFromDeviceHeading(90, -90)).toBe(0);
  });

  it('normalizes bearings across north', () => {
    expect(cameraBearingFromDeviceHeading(267, 90)).toBe(357);
    expect(cameraBearingFromDeviceHeading(93, -90)).toBe(3);
  });

  it('does not invent a bearing for invalid input', () => {
    expect(cameraBearingFromDeviceHeading(Number.NaN, 90)).toBe(0);
  });
});
